#include "ros/ros.h"
#include <tf/tf.h>
#include "tf/transform_listener.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Scalar.h"
#include "tf/LinearMath/Vector3.h"

// Info about OccupancyGrid and Float32MultiArray messages
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"

// Standard C library
#include <cstdlib>
#include <cmath>

// Eigen library and the header that contains my functions
#include <eigen3/Eigen/Core>
#include "ObstacleReconstruction.h"
#include "ObstacleAvoidance.h"
//#include "BezierInterpolation.h"

// To write data into files
#include <fstream>

// Packages to run time related functions
#include <iostream>
#include <chrono>
#include <ctime>

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        /**
         * Publisher that publishes an array containing information about the surface cells of all detected obstacles
         * process_occupancy_grid_node has a subscriber that listens to this topic to retrieve this information and to run the dynamical system
         */
        pub_boundary = n_.advertise<std_msgs::Float32MultiArray>("/boundary_cells", 1);

        /**
         * Publisher that publishes an Occupancy Grid message without data field (state of the cells)
         * Basically the node just copies and publishes the /map topic with an emptied "data" field to just have general information about the
         * occupancy grid like the position of the origin or its dimensions. That way process_occupancy_grid_node can still get information about
         * the map without having to process the very long "data" list that contains the state of all cells (0 or 100).
         */
        pub_map_info = n_.advertise<nav_msgs::OccupancyGrid>("map_info", 1);

        /**
         * Publisher that publishes an Occupancy Grid message containing the state of the map after expansion to take into account the radius of
         * the Ridgeback platform. It contains people that have been added to the map as well. This publisher can be disabled to reduce processing load
         * as this topic is only useful for visualization purpose with Rviz.
         */
        pub_expanded_map = n_.advertise<nav_msgs::OccupancyGrid>("/map_expanded", 1);

        /** Subscriber that listens to the Occupancy grid messages published by the SLAM node */
        sub_map = n_.subscribe("/map", 3, &SubscribeAndPublish::callback_for_map, this);

        /** Subscriber that listens to the PoseArray messages containing the position of all people that are tracked */
        sub_people = n_.subscribe("/pose_people_map_filtered", 2, &SubscribeAndPublish::callback_for_people, this);

        /** Starting time of the node for logging purpose */
        time_start = ros::WallTime::now().toSec();

        /** Default state vector of the robot */
        state_robot << 0, 0, -42;

        /**
         * Flag to initialize the position of the attractor depending on the position of the Ridgeback in the map
         * because the position of the attractor is currently expressed in the frame of the robot when the node is launched.
         * If the position of the attractor is (10,0,0) it means I want the attractor ten meters in front of the Ridgeback when the
         * node starts and not at the position (10,0,0) in the map.
         */
        init_attractor = true;
        drift_odometry_x = 0.0;
        drift_odometry_y = 0.0;
        drift_odometry_yaw = 0.0;

        /** Subscriber that listens to the clock publishes by the main loop of process_occupancy_grid_node to synchronize logging timestamps */
        sub_clock = n_.subscribe("/clock_logging", 1, &SubscribeAndPublish::callback_clock, this);
        clock_from_main_loop = 0.0;

        /** Enable or disable the logging of how much time each "step" of the algorithm takes to have an idea of the repartition of the processing load */
        timing_enabled = true;
        if (timing_enabled)
        {
            std::cout << "Opening timing file" << std::endl;
            my_timing.open("/home/qolo/catkin_ws/src/process_occupancy_grid/src/Logging/timing_refresh_"+std::to_string(static_cast<int>(std::round(time_start)))+".txt", std::ios::out | std::ios_base::app);
        }

        ////////////////
        // PARAMETERS //
        ////////////////

        /** Size of gmapping cells (the one you use for delta in rosrun gmapping slam_gmapping scan:=/scan _delta:=0.2 [...] ) */
        size_cell = 0.2;

        /** Radius of the Ridgeback */
        float radius_ridgeback = 0.6;

        /** Number of cells that obstacles should be expanded by to take into account the radius of the Ridgeback. */
        n_expansion = static_cast<int>(std::ceil(radius_ridgeback/size_cell));

        /**
         * Radius of the security circle around people (in meters)
         * It does not include the radius of the ridgeback ~0.6 meters that is also taken into account.
         */
        float radius_around_people = 1.0;
        radius_in_cells = static_cast<int>(std::ceil(radius_around_people / size_cell)); // radius_around_people is in [m] and we need a value in [cell]

        /** Limit distance to consider obstacles (in meters) */
        float limit_in_meters = 3;
        limit_in_cells  = static_cast<int>(std::ceil(limit_in_meters/size_cell)); // limit_in_meters is in [m] and we need a value in [cell]

        /**
         * Enable or disable the logging of several variables during the experiment to display what happened with the Python interactive script
         * Value of logging_enabled is set in ObstacleReconstruction.cpp as a global variable
         */
        extern bool logging_enabled;
        if (logging_enabled)
        {
            std::cout << "Opening log file" << std::endl;
            mylog.open("/home/qolo/catkin_ws/src/process_occupancy_grid/src/Logging/data_blobs_"+std::to_string(static_cast<int>(std::round(time_start)))+".txt", std::ios::out | std::ios_base::app);
        }
    }

    ~SubscribeAndPublish()
    {
        /** Close the logging file is logging is enabled */
        extern bool logging_enabled;
        if (logging_enabled)
        {
            std::cout << "Closing log file" << std::endl;
            mylog.close();
        }

        /** Close the timing file is timer logging is enabled */
        if (timing_enabled)
        {
            my_timing.close();
        }
    }
    /**
     * Callback function that is triggered when the refreshed occupancy grid from the SLAM algorithm is received (/map topic)
     */
    void callback_for_map(const nav_msgs::OccupancyGrid& input)
    {
        extern bool logging_enabled;        // logging_enabled is set in ObstacleReconstruction.cpp as a global variable
        extern Eigen::MatrixXf log_refresh; // log_refresh is set in ObstacleReconstruction.cpp as a global variable

        auto t_start = std::chrono::high_resolution_clock::now(); // Get starting time of the callback function

        ////////////////////////////////////////////////////////////////
        // PROCESSING MAP INTO SOMETHING THAT CAN BE USED MORE EASILY //
        ////////////////////////////////////////////////////////////////

        ROS_DEBUG("Map received from Gmapping node");
        map_gmapping = input;

        // Retrieve OccupancyGrid data that has been received and put it into a std container
        std::vector<int8_t> test_vector = map_gmapping.data;

        // Occupancy grid is now stored in the vector (1 dimension) with type int8_t
        // Convert std container to type int
        std::vector<int> int_vector(test_vector.begin(), test_vector.end());

        // Get std container into an Eigen matrix with the right width and height
        // Once the occupancy grid has been reshaped, I can use my C++ algorithms with it
        Eigen::Map<Eigen::MatrixXi> eig_test_callback( int_vector.data(), map_gmapping.info.height, map_gmapping.info.width);
        eig_test = eig_test_callback; // OPTIMIZATION: Check if it is required. If not then it is an useless copy

        // Retrieving Resolution and Pose of the map in the world frame
        x_pose = map_gmapping.info.origin.position.x;
        y_pose = map_gmapping.info.origin.position.y;

        //////////////////////////////////////
        // PROCESSING POSITION OF THE ROBOT //
        //////////////////////////////////////

        // Get the transform between /map and /base_link (to get the position of the robot in the map)
        bool has_map = false;
        do
        {
            has_map = false;
            try
            {
                listener_.lookupTransform("map", "base_link", ros::Time(0), transform_);
                ROS_DEBUG("Transform is ready");
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                has_map = true;
            }
        }
        while (has_map);

        // Initialization of the attractor during the first callback
        if (init_attractor)
        {
            init_attractor = false; // Initialize only once

            // Get current position of the robot: translation between "map" and "base_link"
            drift_odometry_x = transform_.getOrigin().getX();
            drift_odometry_y = transform_.getOrigin().getY();

            // Get rotation information between "map" and "base_link"
            tfScalar yaw, pitch, roll;
            tf::Matrix3x3 mat(transform_.getRotation());
            mat.getRPY(roll, pitch, yaw); // Assign values to roll pitch yaw variables
            drift_odometry_yaw = yaw;
            ROS_DEBUG("Robot starting position: %f %f %f", drift_odometry_x, drift_odometry_y, drift_odometry_yaw);
        }

        // Get rotation information between "map" and "base_link"
        tfScalar yaw, pitch, roll;
        tf::Matrix3x3 mat(transform_.getRotation());
        mat.getRPY(roll, pitch, yaw); // Assign values to roll pitch yaw variables

        // Create robot state vector and fill it with the current position of the robot
        state_robot << (transform_.getOrigin().getX() - x_pose)/ size_cell,
                    (transform_.getOrigin().getY() - y_pose)/ size_cell,
                    yaw;
        ROS_DEBUG("Robot position: %f %f %f", state_robot(0,0), state_robot(1,0), state_robot(2,0));

        //////////////////////////////////////////
        // PROCESSING POSITION OF THE ATTRACTOR //
        //////////////////////////////////////////

        // Position of the attractor compared to the starting position of the robot
        float position_goal_world_frame_x = -8.0;//-1.4;
        float position_goal_world_frame_y =  0.0;//2.6;

        // Get the (x,y) coordinates in the world frame of cell (0,0)
        float start_cell_x = (-1 * std::round(map_gmapping.info.origin.position.x / size_cell));
        float start_cell_y = (-1 * std::round(map_gmapping.info.origin.position.y / size_cell));

        // Take into account the starting position of the robot to get the position of the attractor in the map
        float position_goal_world_frame_corrected_x =  position_goal_world_frame_x * std::cos(-drift_odometry_yaw) + position_goal_world_frame_y * std::sin(-drift_odometry_yaw);
        float position_goal_world_frame_corrected_y =  position_goal_world_frame_x * (-1) * std::sin(-drift_odometry_yaw) + position_goal_world_frame_y * std::cos(-drift_odometry_yaw);
        ROS_DEBUG("Attractor position in world: %f %f %f", position_goal_world_frame_corrected_x, position_goal_world_frame_corrected_y, 0.0);

        // Convert the position of the attractor into the occupancy map frame
        float target_cell_x = start_cell_x + std::round((position_goal_world_frame_corrected_x+drift_odometry_x) / size_cell);
        float target_cell_y = start_cell_y + std::round((position_goal_world_frame_corrected_y+drift_odometry_y) / size_cell);

        // Create attractor state vector and fill it with the position of the attractor
        State state_attractor;
        state_attractor << target_cell_x, target_cell_y, 0;
        ROS_DEBUG("Attractor position in occupancy grid: %f %f %f", target_cell_x, target_cell_y, 0.0);

        ////////////////////////////////////
        // SAVING SOME DATA IN LOG MATRIX //
        ////////////////////////////////////

        // Detecting all blobs of occupied cells
        Grid grid_tempo;
        if ((logging_enabled)&&(clock_from_main_loop>0))
        {
            // Storing all occupied cells of the occupancy grid (feature 1)
            for (int i=0; i<eig_test.rows(); i++)
            {
                for (int j=0; j<eig_test.cols(); j++)
                {
                    if (eig_test(i,j)==100) // 100 means the cell is occupied
                    {
                        int n_rows = log_refresh.rows();
                        log_refresh.conservativeResize(n_rows+1, Eigen::NoChange);
                        /*log_refresh.block(n_rows,0,1,1) << 0.0f;
                        log_refresh.block(n_rows,1,1,1) << 1.0f;
                        log_refresh.block(n_rows,2,1,2) << static_cast<float>(i), static_cast<float>(j);
                        log_refresh.block(n_rows,4,1,3) << 0.0f,0.0f,0.0f;*/
                        log_refresh.row(n_rows) << 0.0f, 1.0f, static_cast<float>(i), static_cast<float>(j), 0.0f, 0.0f, 0.0f;
                    }
                }
            }

            // Storing position of the robot in the initial space (feature 5)
            log_refresh.conservativeResize(log_refresh.rows()+1, Eigen::NoChange);
            log_refresh.row(log_refresh.rows()-1) << 0.0, 5.0, state_robot(0,0), state_robot(1,0), 0.0,0.0,0.0;

            // Storing position of the robot in the initial space (feature 6)
            log_refresh.conservativeResize(log_refresh.rows()+1, Eigen::NoChange);
            log_refresh.row(log_refresh.rows()-1) << 0.0, 6.0, state_attractor(0,0), state_attractor(1,0), 0.0,0.0,0.0;
        }

        //////////////////////////////
        // EXPANDING OCCUPANCY GRID //
        //////////////////////////////

        // Expand obstacles to take into account the radius of the Ridgeback and get a security margin around static obstacles
        eig_expanded = expand_occupancy_grid( eig_test, n_expansion, state_robot, limit_in_cells, size_cell);

        if (false) // to display a part of the occupancy map centered on the robot in the console  (before adding people)
        {
           eig_expanded(state_robot(0,0),state_robot(1,0)) = -2; // For visualization purpose, set the cell to -2 the cell the robot is standing on
           eig_expanded(state_attractor(0,0),state_attractor(1,0)) = -3; // For visualization purpose, set the cell to -3 the cell the attractor is standing on
           int corner_square_x = static_cast<int>(std::floor((transform_.getOrigin().getX() - x_pose)/size_cell)-25);
           int corner_square_y = static_cast<int>(std::floor((transform_.getOrigin().getY() - y_pose)/size_cell)-25);
           std::cout << eig_expanded.block( corner_square_x, corner_square_y, 51, 51) << std::endl;
        }

        // Time at the end of this step
        auto t_process_grid = std::chrono::high_resolution_clock::now();

        /////////////////////////////////////
        // ADDING PEOPLE TO OCCUPANCY GRID //
        /////////////////////////////////////

        for (int i_col=0; i_col<detected_people.cols(); i_col++)
        {
            // Converting the position of the person into the occupancy map frame (position in cells)
            float person_cell_x = start_cell_x + std::round(detected_people(0,i_col) / size_cell);
            float person_cell_y = start_cell_y + std::round(detected_people(1,i_col) / size_cell);

            ROS_DEBUG("Adding a disk at position (%f,%f)", person_cell_x, person_cell_y);

            // Drawing a disk centered on the position of the person in the occupancy grid
            draw_circle(eig_expanded, static_cast<int>(person_cell_x), static_cast<int>(person_cell_y), radius_in_cells);

            /* // NOT USED - If pedestrians' velocity estimation is good enough it could be used to take into account their future position
            // Taking into account speed to estimate the position of the person 1 second in the future
            person_cell_x = start_cell_x + std::round((detected_people(0,i_col) + detected_people(2,i_col)*1.0) / size_cell);
            person_cell_y = start_cell_y + std::round((detected_people(1,i_col) + detected_people(3,i_col)*1.0) / size_cell);

            // Drawing a disk centered on the future position of the person in the occupancy grid
            draw_circle(eig_expanded, static_cast<int>(person_cell_x), static_cast<int>(person_cell_y), radius_in_cells);

            ROS_DEBUG("Adding prediction disk at position (%f,%f)", person_cell_x, person_cell_y);
            */
        }

        if (false) // to display a part of the occupancy map centered on the robot in the console (after adding people)
        {
            int corner_square_x = static_cast<int>(std::floor((transform_.getOrigin().getX() - x_pose)/size_cell)-25);
            int corner_square_y = static_cast<int>(std::floor((transform_.getOrigin().getY() - y_pose)/size_cell)-25);
            std::cout << eig_expanded.block( corner_square_x, corner_square_y, 51, 51) << std::endl;
        }

        if (false) // to publish the expanded occupancy grid over the /map_expanded topic for visualization purpose using Rviz
        {
            // Copy the map received from the SLAM algorithm
            nav_msgs::OccupancyGrid output_map = input;

            // Transform the Eigen matrix that contains the expanded occupancy grid into a vector of int8_t
            std::vector<int> vec_data(eig_expanded.size());
            Eigen::Map<Eigen::MatrixXi>(vec_data.data(), eig_expanded.rows(), eig_expanded.cols()) = eig_expanded;
            std::vector<int8_t> vec_final(vec_data.size());
            for (int i=0; i < vec_data.size(); i++)
            {
                vec_final[i] = static_cast<int8_t>(vec_data[i]);
            }

            // Replace the data field with the vector that has just been created which contains the expanded occupancy grid
            output_map.data = vec_final;

            // Publish expanded occupancy grid
            pub_expanded_map.publish(output_map);
        }

        // Time at the end of this step
        auto t_process_people = std::chrono::high_resolution_clock::now();

        /////////////////////////
        // DETECTING OBSTACLES //
        /////////////////////////

        // Detect expanded obstacles (static obstacles + people)
        storage = detect_borders( eig_expanded, state_robot );
        ROS_DEBUG("%i obstacles have been detected near the robot", storage.size());

        // Erase obstacles that are too small to be real obstacles (problem with detection algo)
        /* int num_of_obstacles = storage.size();
        for (int i=num_of_obstacles-1; i >= 0; i--)
        {
            if (storage[i].rows()< (8*n_expansion+4)) { storage.erase(storage.begin()+i);}
        }*/

        // Log reconstructed surfaces of detected obstacles
        if ((logging_enabled)&&(clock_from_main_loop>0))
        {
            for (int i=0; i < storage.size(); i++)
            {
                int current_obstacle = i + 1;
                log_refresh.conservativeResize(log_refresh.rows()+(storage[i]).rows(), Eigen::NoChange); // Add rows at the end
                log_refresh.block(log_refresh.rows()-(storage[i]).rows(), 0, (storage[i]).rows(), 1) = current_obstacle * Eigen::MatrixXf::Ones((storage[i]).rows(), 1); // Numero of obstacle
                log_refresh.block(log_refresh.rows()-(storage[i]).rows(), 1, (storage[i]).rows(), 1) = 2 * Eigen::MatrixXf::Ones((storage[i]).rows(), 1); // Numero of feature
                log_refresh.block(log_refresh.rows()-(storage[i]).rows(), 2, (storage[i]).rows(), 5) = storage[i]; // Add border information
            }
        }

        //////////////////////////////////////////////////////////////
        // WRAPPING RECONSTRUCTED SURFACES INTO A Float32MultiArray //
        //////////////////////////////////////////////////////////////

        /**
         * The information about reconstructed surfaces needs to be transmitted to process_occupancy_grid_node
         * This information is contained in a vector of Eigen matrices but ROS only supports the transmission of standard vectors
         * As a result, we need to wrap this information in the shape of a standard vector that will be unwrapped by process_occupancy_grid_node
         * to get back a vector of Eigen matrices
         */

        // Standard ROS message - Array with several dimensions
        std_msgs::Float32MultiArray array_msg;

        /**
         * Get the maximum number of row among all Eigen matrices
         * When they are transmitted in the std_msgs::Float32MultiArray, all obstacles needs to have the same number of rows
         * so we get the number of rows of the biggest obstacles. Later empty rows will be added to smaller obstacles to make
         * them have the number of rows of the biggest obstacle.
         */
        int max_num_row = 0;
        for (int i=0; i<storage.size(); i++)
        {
            if ((storage[i]).rows() > max_num_row)
            {
                max_num_row = (storage[i]).rows();
            }
        }

        // Padding at the beginning of the array (no padding)
        array_msg.layout.data_offset = 0;

        // Set up dimensions
        array_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        array_msg.layout.dim[0].label = "obstacles";
        array_msg.layout.dim[0].size = storage.size();
        array_msg.layout.dim[0].stride = 5*max_num_row*storage.size();
        array_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        array_msg.layout.dim[1].label = "rows";
        array_msg.layout.dim[1].size = max_num_row;
        array_msg.layout.dim[1].stride = 5*max_num_row;
        array_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        array_msg.layout.dim[2].label = "columns";
        array_msg.layout.dim[2].size = 5;
        array_msg.layout.dim[2].stride = 5;

        // Fill data field
        for (int i=0; i<storage.size(); i++)
        {
            ROS_DEBUG("Obstacle %i has %i cells", i, (storage[i]).rows());
            Border tempo = storage[i];
            for (int j=0; j<max_num_row; j++)
            {
                for (int k=0; k<5; k++)
                {
                    if (j<(storage[i]).rows())
                    {
                        array_msg.data.push_back(tempo(j,k));
                    }
                    else
                    {
                        array_msg.data.push_back(0.0); // Padding at the end with empty rows
                    }
                }
            }
        }

        // Time at the end of this step
        auto t_detect_obs = std::chrono::high_resolution_clock::now();

        ////////////////////////////
        // LOGGING AND PUBLISHING //
        ////////////////////////////

        if ((timing_enabled)&&(clock_from_main_loop>0))
        {
            std::chrono::duration<double> diff = t_process_grid - t_start;
            //std::cout << "T process grid:      " << diff.count() << std::endl;
            my_timing << 5 << "," << clock_from_main_loop << "," << diff.count() << "\n";

            diff = t_process_people - t_process_grid;
            //std::cout << "T adding people:     " << diff.count() << std::endl;
            my_timing << 4 << "," << clock_from_main_loop << "," << 0.0 << "\n";

            diff = t_detect_obs - t_process_people;
            //std::cout << "T detect obstacles:  " << diff.count() << std::endl;
            my_timing << 6 << "," << clock_from_main_loop << "," << diff.count() << "\n";
        }

        // Publish array contained reconstructed surfaces that will be received by process_occupancy_grid_node
        pub_boundary.publish(array_msg);

        // Empty OccupancyGrid message with just the metadata of the OccupancyGrid received by the SLAM algorithm
        nav_msgs::OccupancyGrid map_just_info;
        map_just_info.header = input.header;
        map_just_info.info = input.info;
        // map_just_info.data is left empty

        // Publish metadata of the occupancy grid that will be received by process_occupancy_grid_node
        pub_map_info.publish(map_just_info);

        // Saving data in log_refresh then writing to text file with CSV format
        if ((logging_enabled)&&(clock_from_main_loop>0))
        {
            float timestamp = clock_from_main_loop;
            Eigen::MatrixXf time_matrix = timestamp * Eigen::MatrixXf::Ones(log_refresh.rows(),1);
            Eigen::MatrixXf matrix(log_refresh.rows(), 1+log_refresh.cols());
            matrix << time_matrix, log_refresh;

            const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
            mylog << matrix.format(CSVFormat) << "\n";
            log_refresh = Eigen::MatrixXf::Zero(1,7);
        }
    }


    void callback_clock(const std_msgs::Float32& input)
    {
        clock_from_main_loop = input.data;
    }

    void callback_for_people(const geometry_msgs::PoseArray& people) // Callback triggered by the /pose_people_map topic
    {
    Eigen::MatrixXf temp_storage;

    for (int i=0; i<people.poses.size(); i++)
    {
        temp_storage.conservativeResize(10, temp_storage.cols()+1); // add a column to store the new detected person


        // TODO: test implementation of tranformPose once it is done
        geometry_msgs::PoseStamped pose_person;
        pose_person.header = people.header;
        pose_person.pose = people.poses[i];


        //geometry_msgs::PoseStamped pose_person_in_map;
        //pose_person_in_map.header = people.header;
        //listener.tranformPose("map", pose_person, pose_person_in_map)

        float received_time = (pose_person.header.stamp).toSec();
        float personX = pose_person.pose.position.x;
        float personY = pose_person.pose.position.y;
        float personZ = pose_person.pose.position.z;

        // Check if it's (X,Z) and not (Z,X) or (X,Y)...
        // [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot]
        // temp_storage.col(i) << personX, personY, 0, 1, 1, 1, 1, 0, 0, 0; // circle centered on the person

        // [x_c, y_c, v_x, v_y, stamp, 0, 0, 0, 0, 0]
        if (detected_people.cols() >= (i+1))
	{
             if (received_time != detected_people(4,i))
             {
	         float dx, dy, dt;
                 dx = personX - detected_people(0,i);
                 dy = personY - detected_people(1,i);
                 dt = received_time - detected_people(4,i);
	         temp_storage.col(i) << personX, personY, (dx/dt), (dy/dt), received_time, 0, 0, 0, 0, 0;
             }
             else
             {
                  temp_storage.col(i) << personX, personY, detected_people(2,i), detected_people(3,i), received_time, 0, 0, 0, 0, 0;
             }
        }
        else
        {
             temp_storage.col(i) << personX, personY, 0, 0, received_time, 0, 0, 0, 0, 0; // no previous information to compute velocity
        }



    }
    detected_people = temp_storage;
    //std::cout << "Detected people :" << std::endl << detected_people.block(0,0,2,2) << std::endl;
}

private:
    ros::NodeHandle n_;
    ros::Publisher pub_boundary;  // To publish the velocity command
    ros::Publisher pub_map_info;  // To publish map info for main loop
    ros::Publisher pub_expanded_map;  // To publish map info for main loop
    ros::Subscriber sub_map;    // To listen to the map topic
    ros::Subscriber sub_people; // To listen to the pose_people topic
    tf::TransformListener listener_; // To listen to transforms
    tf::StampedTransform transform_; // Transform from map to base_link

    Eigen::MatrixXf detected_people;
    float time_start; // to know when the node has been launched
    std::ofstream mylog;
    std::ofstream my_timing;

    State state_robot;

    nav_msgs::OccupancyGrid map_gmapping;
    Eigen::MatrixXi eig_test;
    Eigen::MatrixXi eig_expanded;
    float x_pose;
    float y_pose;
    std::vector<Border> storage;
    int n_expansion;
    int limit_in_cells;
    float size_cell;
    int   radius_in_cells;

    bool init_attractor;
    float drift_odometry_x;
    float drift_odometry_y;
    float drift_odometry_yaw;

    float clock_from_main_loop;
    ros::Subscriber sub_clock;

    bool timing_enabled;
};//End of class SubscribeAndPublish



int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "refresh_occupancy_grid_node");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    // Spin node
    ros::spin();

    return 0;
}


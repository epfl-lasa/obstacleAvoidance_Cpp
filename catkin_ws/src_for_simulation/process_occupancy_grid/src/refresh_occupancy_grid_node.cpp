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
        // Topic you want to publish (array containing boundary cells)
        pub_boundary = n_.advertise<std_msgs::Float32MultiArray>("/boundary_cells", 1);

        // Topic you want to publish (Occupancy Grid without data field)
        pub_map_info = n_.advertise<nav_msgs::OccupancyGrid>("map_info", 1);

        // Topic you want to publish (Occupancy Grid expanded)
        pub_expanded_map = n_.advertise<nav_msgs::OccupancyGrid>("/map_expanded", 1);

        // Topic you want to subscribe (map topic to avoid service call)
        sub_map = n_.subscribe("/map", 3, &SubscribeAndPublish::callback_for_map, this);

        //Another topic you want to subscribe (position of detected people)
        sub_people = n_.subscribe("/pose_people_map_filtered", 2, &SubscribeAndPublish::callback_for_people, this);

        // Time start of node
        time_start = ros::WallTime::now().toSec();

        // State vector of the robot
        state_robot << 0, 0, -42;

        // Flag to initialize the position of the attractor depending on the initial position of the Ridgeback in the map
       // Because of odometry drift the Ridgeback never starts at the same position in the map.
       init_attractor = true;
       drift_odometry_x = 0.0;
       drift_odometry_y = 0.0;
       drift_odometry_yaw = 0.0;

        // Listening to the clock emitted by the main loop to synchronise logging
        sub_clock = n_.subscribe("/clock_logging", 1, &SubscribeAndPublish::callback_clock, this);
        clock_from_main_loop = 0.0;

        timing_enabled = true;

        ////////////////
        // PARAMETERS //
        ////////////////

        // Size of gmapping cells (the one you use for delta in rosrun gmapping slam_gmapping scan:=/scan _delta:=0.3 _map_update_interval:=1.0)
        size_cell = 0.2;

        // Radius of the Ridgeback
        float radius_ridgeback = 0.6;

        // Number of cells that the obstacles should be expanded
        n_expansion = static_cast<int>(std::ceil(radius_ridgeback/size_cell));

        // Radius of the security circle around people (in meters)
        // It does not include the radius of the ridgeback ~0.6 meters (disk will be expanded)
        float radius_around_people = 0.5;
        radius_in_cells = static_cast<int>(std::ceil(radius_around_people / size_cell)); // radius_around_people is in [m] and we need a value in [cell]

        // Limit distance to consider obstacles (in meters)
        float limit_in_meters = 3;
        limit_in_cells  = static_cast<int>(std::ceil(limit_in_meters/size_cell));

        extern bool logging_enabled;
	if (logging_enabled)
	{
             std::cout << "Opening log file" << std::endl;
	     mylog.open("/home/qolo/catkin_ws/src/process_occupancy_grid/src/Logging/data_blobs_"+std::to_string(static_cast<int>(std::round(time_start)))+".txt", std::ios::out | std::ios_base::app);
	}

        if (timing_enabled)
        {
        std::cout << "Opening timing file" << std::endl;
        my_timing.open("/home/qolo/catkin_ws/src/process_occupancy_grid/src/Logging/timing_refresh_"+std::to_string(static_cast<int>(std::round(time_start)))+".txt", std::ios::out | std::ios_base::app);
        }

    }

    ~SubscribeAndPublish()
    {
        // Destructor
        extern bool logging_enabled;
        if (logging_enabled)
        {
            std::cout << "Closing log file" << std::endl;
            mylog.close();
        }
	if (timing_enabled)
	{
	    my_timing.close();
	}
    }

    void callback_for_map(const nav_msgs::OccupancyGrid& input) // Callback triggered by /map topic
    {
	//std::cout << "Clock: " << clock_from_main_loop << std::endl;
        extern bool logging_enabled;
        extern Eigen::MatrixXf log_refresh;

        //float timestamp_timing = ros::Time::now().toSec();

        auto t_start = std::chrono::high_resolution_clock::now();

        ////////////////////////////////////////////////////////////////
        // PROCESSING MAP INTO SOMETHING THAT CAN BE USED MORE EASILY //
        ////////////////////////////////////////////////////////////////

        // std::cout << " == Map received from Gmapping node == " << std::endl;
        map_gmapping = input;

        // Retrieving OccupancyGrid data that has been received
        std::vector<int8_t> test_vector = map_gmapping.data; // get map into a std container

        // Occupancy grid is now stored in the vector (1 dimension) with type int8_t
        // Convert std container to type int
        std::vector<int> int_vector(test_vector.begin(), test_vector.end());

        // Get std container into an Eigen matrix with the right width and height
        Eigen::Map<Eigen::MatrixXi> eig_test_callback( int_vector.data(), map_gmapping.info.height, map_gmapping.info.width);
        eig_test = eig_test_callback;
        // The occupancy grid has been reshaped to be able to use C++ algorithms with it

        // Retrieving Resolution and Pose of the map in the world frame
        x_pose = map_gmapping.info.origin.position.x;
        y_pose = map_gmapping.info.origin.position.y;

        //////////////////////////////////////
        // PROCESSING POSITION OF THE ROBOT //
        //////////////////////////////////////


	/*while (!listener_.canTransform("base_link", "map", ros::Time(0)))
	{
		try
		{
		        listener_.lookupTransform("map", "base_link", ros::Time(0), transform_);
		        ROS_INFO("Transform is ready");
		}
		ROS_INFO("Cannot transform from base_link to map");
	}
        ROS_INFO("Transform is ready");*/

        // Get the transform between /map and /base_link (to get the position of the robot in the map)
        bool has_map = false;
        do
        {
            has_map = false;
            try
            {
                listener_.lookupTransform("map", "base_link", ros::Time(0), transform_);
                // ROS_INFO("Transform is ready");
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                has_map = true;
            }
        }
        while (has_map);

         if (init_attractor)
         {
            init_attractor = false;
            drift_odometry_x = transform_.getOrigin().getX();
            drift_odometry_y = transform_.getOrigin().getY();
            // Get rotation information between "map" and "base_link"
            tfScalar yaw, pitch, roll;
            tf::Matrix3x3 mat(transform_.getRotation());
            mat.getRPY(roll, pitch, yaw); // Assign values to roll pitch yaw variables
            drift_odometry_yaw = yaw;
            //ROS_INFO("Drift     | %f %f %f", drift_odometry_x, drift_odometry_y, drift_odometry_yaw);
         }

        // Get rotation information between "map" and "base_link"
        tfScalar yaw, pitch, roll;
        tf::Matrix3x3 mat(transform_.getRotation());
        mat.getRPY(roll, pitch, yaw); // Assign values to roll pitch yaw variables

        // Create robot state vector and fill it
        state_robot << (transform_.getOrigin().getX() - x_pose)/ size_cell,
                    (transform_.getOrigin().getY() - y_pose)/ size_cell,
                    yaw;


        //ROS_INFO("Robot     | %f %f %f", state_robot(0,0), state_robot(1,0), state_robot(2,0));

       //////////////////////////////////////////
       // PROCESSING POSITION OF THE ATTRACTOR //
       //////////////////////////////////////////

       float position_goal_world_frame_x = -5.0;//-1.4;
       float position_goal_world_frame_y =  0.0;//2.6;

       // Get the (x,y) coordinates in the world frame of the cell (0,0)
       float start_cell_x = (-1 * std::round(map_gmapping.info.origin.position.x / size_cell));
       float start_cell_y = (-1 * std::round(map_gmapping.info.origin.position.y / size_cell));

       float position_goal_world_frame_corrected_x =  position_goal_world_frame_x * std::cos(-drift_odometry_yaw) + position_goal_world_frame_y * std::sin(-drift_odometry_yaw);
       float position_goal_world_frame_corrected_y =  position_goal_world_frame_x * (-1) * std::sin(-drift_odometry_yaw) + position_goal_world_frame_y * std::cos(-drift_odometry_yaw);
       //std::cout << position_goal_world_frame_corrected_x << " " << position_goal_world_frame_corrected_y << std::endl;
 
       // Convert the position of the attractor into the occupancy map frame
       float target_cell_x = start_cell_x + std::round((position_goal_world_frame_corrected_x+drift_odometry_x) / size_cell);
       float target_cell_y = start_cell_y + std::round((position_goal_world_frame_corrected_y+drift_odometry_y) / size_cell);

       // Set state of the attractor
       State state_attractor;
       state_attractor << target_cell_x, target_cell_y, 0;
       //ROS_INFO("Attractor | %f %f %f", target_cell_x, target_cell_y, 0.0);

        ////////////////////////////////////
        // SAVING SOME DATA IN LOG MATRIX //
        ////////////////////////////////////

	// Detecting all blobs of occupied cells
        Grid grid_tempo;
        if ((logging_enabled)&&(clock_from_main_loop>0))
        {
           /*grid_tempo = eig_test;
        
        // Storing all occupied cells of the occupancy grid
        std::vector<Blob> storage_blobs;
        storage_blobs = detect_blobs( grid_tempo );
        for (int i=0; i<storage_blobs.size(); i++)
        {
           int n_rows = log_refresh.rows();
           log_refresh.conservativeResize(n_rows+storage_blobs[i].rows(), Eigen::NoChange);
           log_refresh.block(n_rows,0,storage_blobs[i].rows(),1) = Eigen::MatrixXf::Zero(storage_blobs[i].rows(),1);
           log_refresh.block(n_rows,1,storage_blobs[i].rows(),1) = Eigen::MatrixXf::Ones(storage_blobs[i].rows(),1);
           log_refresh.block(n_rows,2,storage_blobs[i].rows(),2) = ((storage_blobs[i]).block(0,0,storage_blobs[i].rows(),2)).template cast<float>();
           log_refresh.block(n_rows,4,storage_blobs[i].rows(),3) = Eigen::MatrixXf::Zero(storage_blobs[i].rows(),3);
        }*/

        for (int i=0; i<eig_test.rows(); i++)
        {
            for (int j=0; j<eig_test.cols(); j++) 
	    {
                 if (eig_test(i,j)==100)
                 {
                     int n_rows = log_refresh.rows();
                     log_refresh.conservativeResize(n_rows+1, Eigen::NoChange);
                     log_refresh.block(n_rows,0,1,1) << 0.0f;
                     log_refresh.block(n_rows,1,1,1) << 1.0f;
                     log_refresh.block(n_rows,2,1,2) << static_cast<float>(i), static_cast<float>(j);
                     log_refresh.block(n_rows,4,1,3) << 0.0f,0.0f,0.0f;
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

        ///////////////////////////////
        // PROCESSING OCCUPANCY GRID //
        ///////////////////////////////

        //std::cout << " PROCESSING OCCUPANCY GRID " << std::endl;

        // Expand obstacles to get a security margin
        eig_expanded = expand_occupancy_grid( eig_test, n_expansion, state_robot, limit_in_cells, size_cell);


        if (false) // to display a part of the occupancy map centered on the robot in the console
        {
           eig_expanded(state_robot(0,0),state_robot(1,0)) = -2;
           eig_expanded(state_attractor(0,0),state_attractor(1,0)) = -3;
           int corner_square_x = static_cast<int>(std::floor((transform_.getOrigin().getX() - x_pose)/size_cell)-25);
           int corner_square_y = static_cast<int>(std::floor((transform_.getOrigin().getY() - y_pose)/size_cell)-25);
           std::cout << eig_expanded.block( corner_square_x, corner_square_y, 51, 51) << std::endl;
        }

        auto t_process_grid = std::chrono::high_resolution_clock::now();
        //ROS_INFO("Occupancy grid has been processed");

    /////////////////////////////////////
    // ADDING PEOPLE TO OCCUPANCY GRID //
    /////////////////////////////////////

    //Eigen::MatrixXi eig_people;// = eig_test;
    
    for (int i_col=0; i_col<detected_people.cols(); i_col++)
    {
        // Converting the position of the person into the occupancy map frame
        float person_cell_x = start_cell_x + std::round(detected_people(0,i_col) / size_cell);
        float person_cell_y = start_cell_y + std::round(detected_people(1,i_col) / size_cell);

        //ROS_INFO("Drawing a circle at position (%f,%f)", person_cell_x, person_cell_y);

	// Drawing a disk centered on the position of the person in the occupancy grid
        draw_circle(eig_expanded, static_cast<int>(person_cell_x), static_cast<int>(person_cell_y), radius_in_cells);

        /*// Taking into account speed to estimate the position of the person 1 second in the future
        person_cell_x = start_cell_x + std::round((detected_people(0,i_col) + detected_people(2,i_col)*1.0) / size_cell);
        person_cell_y = start_cell_y + std::round((detected_people(1,i_col) + detected_people(3,i_col)*1.0) / size_cell);

        // Drawing a disk centered on the future position of the person in the occupancy grid
        draw_circle(eig_expanded, static_cast<int>(person_cell_x), static_cast<int>(person_cell_y), radius_in_cells);

        ROS_INFO("Drawing prediction at position (%f,%f)", person_cell_x, person_cell_y);*/
    }

    if (false) // enable to display a part of the occupancy map centered on the robot in the console
    {
        int corner_square_x = static_cast<int>(std::floor((transform_.getOrigin().getX() - x_pose)/size_cell)-25);
        int corner_square_y = static_cast<int>(std::floor((transform_.getOrigin().getY() - y_pose)/size_cell)-25);
        std::cout << eig_expanded.block( corner_square_x, corner_square_y, 51, 51) << std::endl;
    }

        /*
        //std::vector<int8_t> vec_data;

        //Eigen::VectorXi vec_xi(Eigen::Map<Eigen::VectorXi>(eig_expanded.data(),eig_expanded.rows()*eig_expanded.cols()));

	//vec_data.resize(vec_xi.size());
        //Eigen::VectorXi::Map(&vec_data[0],vec_xi.size()) = vec_xi;

        //std::vector<int> vec_data(&vec_xi[0],vec_xi.data()+vec_xi.rows()*vec_xi.cols());*/

        nav_msgs::OccupancyGrid output_map = input;
	std::vector<int> vec_data(eig_expanded.size());
        Eigen::Map<Eigen::MatrixXi>(vec_data.data(), eig_expanded.rows(), eig_expanded.cols()) = eig_expanded;

        std::vector<int8_t> vec_final(vec_data.size());
        for (int i=0; i < vec_data.size(); i++)
        {
             vec_final[i] = static_cast<int8_t>(vec_data[i]);
        }

        output_map.data = vec_final;
        pub_expanded_map.publish(output_map);

        /////////////////////////
        // DETECTING OBSTACLES //
        /////////////////////////

        //std::cout << " DETECTING OBSTACLES " << std::endl;

        // Detect expanded obstacles
        storage = detect_borders( eig_expanded, state_robot );

        //std::cout << storage.size() << " have been detected near the robot" << std::endl;

        // Erase obstacles that are too small to be real obstacles (problem with detection algo)
	int num_of_obstacles = storage.size()  ;     
	/*for (int i=num_of_obstacles-1; i >= 0; i--)
        {
            if (storage[i].rows()< (8*n_expansion+4)) { storage.erase(storage.begin()+i);}
        }*/ 

        for (int i=0; i < storage.size(); i++)
        {
        if ((logging_enabled)&&(clock_from_main_loop>0))
        {   
            int current_obstacle = i + 1;
            //std::cout << "Logging border information " << i << std::endl;
            log_refresh.conservativeResize(log_refresh.rows()+(storage[i]).rows(), Eigen::NoChange); // Add rows at the end
            log_refresh.block(log_refresh.rows()-(storage[i]).rows(), 0, (storage[i]).rows(), 1) = current_obstacle * Eigen::MatrixXf::Ones((storage[i]).rows(), 1); // Numero of obstacle
            log_refresh.block(log_refresh.rows()-(storage[i]).rows(), 1, (storage[i]).rows(), 1) = 2 * Eigen::MatrixXf::Ones((storage[i]).rows(), 1); // Numero of feature
            log_refresh.block(log_refresh.rows()-(storage[i]).rows(), 2, (storage[i]).rows(), 5) = storage[i]; // Add border information
        }
        }
        //auto t_detect_obs = std::chrono::high_resolution_clock::now();

        std_msgs::Float32MultiArray array_msg;

        int max_num_row = 0;
        for (int i=0; i<storage.size(); i++)
        {
            if ((storage[i]).rows() > max_num_row) {max_num_row = (storage[i]).rows();}
        }

        //std::cout << " PASS " << std::endl;

        // Padding at the beginning of the array
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
            /*Eigen::MatrixXf coeff = Eigen::MatrixXf::Zero(1,1);
             
            coeff(0,0) = (storage[i]).col(0).minCoeff();
	    float minx = coeff(0,0);            
            coeff(0,0) = (storage[i]).col(0).maxCoeff();
            float maxx = coeff(0,0); 
	    coeff(0,0) = (storage[i]).col(1).minCoeff();
            float miny = coeff(0,0);
	    coeff(0,0) = (storage[i]).col(1).maxCoeff();
            float maxy =  coeff(0,0);

            Eigen::MatrixXi temporaire = Eigen::MatrixXi::Zero(static_cast<int>(maxx-minx+1), static_cast<int>(maxy-miny+1));
            for (int j=0; j<(storage[i]).rows(); j++)
            {
                 temporaire(static_cast<int>((storage[i])(j,0)-minx),static_cast<int>((storage[i])(j,1)-miny)) = 1;
            }
            std::cout << temporaire << std::endl;*/

            //std::cout << "Obstacle " << i << " has " << (storage[i]).rows() << "cells" << std::endl;
            //std::cout << (storage[i]).block(0,0,5,5) << std::endl;
            Border tempo = storage[i];
	    for (int j=0; j<max_num_row; j++)
            {
                for (int k=0; k<5; k++)
                {
                    if (j<(storage[i]).rows())
                    {
			//Border tempo = storage[i];
                        array_msg.data.push_back(tempo(j,k));
                    }
                    else
                    {
                        array_msg.data.push_back(0.0);
                    }
                }
            }
        }
        
        auto t_detect_obs = std::chrono::high_resolution_clock::now();

        /*for (int i=0; i<storage.size(); i++)
        {
            std::cout << storage[i] << std::endl;
        }*/

        if ((timing_enabled)&&(clock_from_main_loop>0))
        {
             //diff = t_adding_people - t_process_attractor;
             //std::cout << "T adding people:     " << diff.count() << std::endl;
             my_timing << 4 << "," << clock_from_main_loop << "," << 0.0 << "\n";

             std::chrono::duration<double> diff = t_process_grid - t_start;
             //std::cout << "T process grid:      " << diff.count() << std::endl;
             my_timing << 5 << "," << clock_from_main_loop << "," << diff.count() << "\n";
             diff = t_detect_obs - t_process_grid;
             //std::cout << "T detect obstacles:  " << diff.count() << std::endl;
             my_timing << 6 << "," << clock_from_main_loop << "," << diff.count() << "\n";
        }

        // Publish message
        pub_boundary.publish(array_msg);

        nav_msgs::OccupancyGrid map_just_info;
	map_just_info.header = input.header;
	map_just_info.info = input.info;
        
        // map_just_info.data is left empty
        // Publish message
        pub_map_info.publish(map_just_info);

    // Saving data in log_refresh
    if ((logging_enabled)&&(clock_from_main_loop>0))
    {
        float timestamp = clock_from_main_loop;//ros::Time::now().toSec() - time_start;
	Eigen::MatrixXf time_matrix = timestamp * Eigen::MatrixXf::Ones(log_refresh.rows(),1);
        Eigen::MatrixXf matrix(log_refresh.rows(), 1+log_refresh.cols());
        matrix << time_matrix, log_refresh;
	//std::cout << log_refresh << std::endl;

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


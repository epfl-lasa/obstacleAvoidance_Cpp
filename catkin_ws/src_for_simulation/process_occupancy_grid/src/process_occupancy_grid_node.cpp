#include "ros/ros.h"
#include <tf/tf.h>
#include "tf/transform_listener.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Scalar.h"
#include "tf/LinearMath/Vector3.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float32MultiArray.h"
//#include "obstacle_avoidance/GetObstacles.h"
#include <cstdlib>
#include <cmath>

// Eigen library and the header that contains my functions
#include <eigen3/Eigen/Core>
#include "ObstacleReconstruction.h"
#include "ObstacleAvoidance.h"
#include "BezierInterpolation.h"

#include <fstream>  // To write data into files

// Packages to run time related functions
#include <iostream>
#include <chrono>
#include <ctime>

typedef Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXi8; // Dynamic Eigen matrix with type int8_t since OccupancyGrid contains int8_t values
typedef Eigen::Matrix<int8_t, 1, Eigen::Dynamic> MatrixXi8_layer;
typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> myMap;


class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish (velocity command for the robot)
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    //Topic you want to publish (velocity command for the robot)
    pub_attractor = n_.advertise<geometry_msgs::PointStamped>("/attractor", 10);

    //Topic you want to publish (logging clock for log_data_robot_node)
    pub_clock = n_.advertise<std_msgs::Float32>("/clock_logging", 10);

    //Topic you want to subscribe (trigger signal)
    sub_ = n_.subscribe("/test", 1, &SubscribeAndPublish::callback, this);

    //Another topic you want to subscribe (position of detected people)
    sub_people = n_.subscribe("/pose_people_map", 2, &SubscribeAndPublish::callback_for_people, this);

    //Another topic you want to subscribe (attractor control with keyboard)
    sub_key = n_.subscribe("/cmd_key", 2, &SubscribeAndPublish::callback_for_key, this);

    //Another topic you want to subscribe (map topic to avoid service call)
    // sub_map = n_.subscribe("/map", 2, &SubscribeAndPublish::callback_for_map, this);
    sub_boundary = n_.subscribe("/boundary_cells", 1, &SubscribeAndPublish::callback_for_boundary, this);

    //Another topic you want to subscribe (map_info topic)
    sub_map_info = n_.subscribe("/map_info", 1, &SubscribeAndPublish::callback_for_map_info, this);
    // /map_metadata is already published by gmapping but need to adapt code for nav_msgs::Metadata

    // Client of dynamic_map service of gmapping (to get the occupancy grid)
    client_map_ = n_.serviceClient<nav_msgs::GetMap>("dynamic_map");

    // Transform listener
    //tf::TransformListener listener_(ros::Duration(10.0));

    // Default command
    key_command = -1;

    // Default attractor
    num_attractor = 0;

    // Default frame number
    ID_frame = 0;

    // Time start of node
    time_start = ros::WallTime::now().toSec();
    time_start_clock = ros::Time::now().toSec();

    // Flag to initialize the position of the attractor depending on the initial position of the Ridgeback in the map
    // Because of odometry drift the Ridgeback never starts at the same position in the map.
    init_attractor = true;
    drift_odometry_x = 0.0;
    drift_odometry_y = 0.0;
    drift_odometry_yaw = 0.0;

    state_robot << 0, 0, -42;

    ////////////////
    // PARAMETERS //
    ////////////////

    // Size of gmapping cells (the one you use for delta in rosrun gmapping slam_gmapping scan:=/scan _delta:=0.3 _map_update_interval:=1.0)
    size_cell = 0.2;

    // Radius of the Ridgeback
    float radius_ridgeback = 0.6;

    // Number of cells that the obstacles should be expanded
    n_expansion = static_cast<int>(std::ceil(radius_ridgeback/size_cell));

    // Limit distance to consider obstacles (in meters)
    float limit_in_meters = 3;
    limit_in_cells  = static_cast<int>(std::ceil(limit_in_meters/size_cell));

    extern bool logging_enabled;
    if (logging_enabled)
    {
        std::cout << "Opening log file" << std::endl;
        mylog.open("/home/qolo/catkin_ws/src/process_occupancy_grid/src/Logging/data_obstacles_"+std::to_string(static_cast<int>(std::round(time_start)))+".txt", std::ios::out | std::ios_base::app);
    }

    if (true)
    {
        std::cout << "Opening timing file" << std::endl;
        my_timing.open("/home/qolo/catkin_ws/src/process_occupancy_grid/src/Logging/timing_functions_"+std::to_string(static_cast<int>(std::round(time_start)))+".txt", std::ios::out | std::ios_base::app);
    }

  }

  ~SubscribeAndPublish()
  {
     extern bool logging_enabled;
     if (logging_enabled)
     {
         std::cout << "Closing log file" << std::endl;
         mylog.close();
     }
     if (true)
     {
         my_timing.close();
     }
  }

  void callback(const geometry_msgs::Twist& input) // callback is triggered by /test topic
  {

     auto t_start = std::chrono::high_resolution_clock::now();


    ////////////////
    // PARAMETERS //
    ////////////////



    // Position of the attractor compared to the initial position of the robot (in meters)
    // Consider that the robot starts at position (0,0)
    // If you set the attractor at (12,-11) the target is 12m forward and 11m to the right of the starting position
    //float position_goal_world_frame_x =  12;// 2.52; //  42; // 23.6; //  15;
    //float position_goal_world_frame_y = -11;//-2.56;//-7.3; //5   ; // 0;

    int max_num_attractor = 4;
    float position_goal_world_frame_x = -8.0;//-1.4; // -2.7
    float position_goal_world_frame_y =  0.0;// 2.6;
    /*switch (num_attractor)
    {
     case 0:
	position_goal_world_frame_x =    42;
	position_goal_world_frame_y =  -7.3; break;
     case 1:
	position_goal_world_frame_x =    40.6;
	position_goal_world_frame_y =  - 36.8; break;
     case 2:
	position_goal_world_frame_x =    1.2;
	position_goal_world_frame_y =  -37; break;
     case 3:
	position_goal_world_frame_x =    -4.8;
	position_goal_world_frame_y =   -10.6; break;
    }*/



    // Radius of the security circle around people (in meters)
    // It does not include the radius of the ridgeback ~0.6 meters (disk will be expanded)
    float radius_around_people = 1.0;
    int   radius_in_cells = static_cast<int>(std::ceil(radius_around_people / size_cell)); // radius_around_people is in [m] and we need a value in [cell]

    // Variables used for logging when logging_enabled is set to true (in ObstacleReconstruction.cpp)
    extern bool logging_enabled;
    extern Eigen::MatrixXf log_matrix;
    //Eigen::MatrixXf log_matrix = Eigen::MatrixXf::Zero(1,7); // Matrix to store all the log data

    auto t_parameters = std::chrono::high_resolution_clock::now();
    //////////////////////////////////
    // RETRIEVING MAP FROM GMAPPING //
    //////////////////////////////////

    /* MAP IS NOW RECEIVED WITH CALLBACK_FOR_MAP
    // Call gmapping service to get the occupancy grid
    nav_msgs::GetMap srv_map;
    if (client_map_.call(srv_map))
    {
        ROS_INFO("Dynamic map received"); //map is in srv.response.map
    }
    else
    {
       ROS_ERROR("Failed to call dynamic map service");
    }

    // Retrieving OccupancyGrid data that has been received
    std::vector<int8_t> test_vector = (srv_map.response.map.data); // get map into a std container


    // Occupancy grid is now stored in the vector (1 dimension) with type int8_t
    // Convert std container to type int
    std::vector<int> int_vector(test_vector.begin(), test_vector.end());

    // Get std container into an Eigen matrix with the right width and height
    Eigen::Map<Eigen::MatrixXi> eig_test( int_vector.data(), srv_map.response.map.info.height, srv_map.response.map.info.width);

    // The occupancy grid has been reshaped to be able to use C++ algorithms with it

    // Retrieving Resolution and Pose of the map in the world frame
    float resolution = srv_map.response.map.info.resolution;
    float x_pose = srv_map.response.map.info.origin.position.x;
    float y_pose = srv_map.response.map.info.origin.position.y;
    // ROS_INFO("Map origin: %f %f", x_pose, y_pose);*/

    // NO NEED SINCE THE MAP IS USED AS THE REFERENCE FRAME
    /*tfScalar roll, pitch, yaw;
    tf::Matrix3x3 mat(srv_map.response.map.info.origin.orientation);
    mat.getRPY(roll, pitch, yaw);
    float phi_pose = yaw;*/
    //////

    /*
    // Retrieving OccupancyGrid data that has been received
    std::vector<int8_t> test_vector = map_gmapping.data; // get map into a std container

    // Occupancy grid is now stored in the vector (1 dimension) with type int8_t
    // Convert std container to type int
    std::vector<int> int_vector(test_vector.begin(), test_vector.end());

    // Get std container into an Eigen matrix with the right width and height
    Eigen::Map<Eigen::MatrixXi> eig_test( int_vector.data(), map_gmapping.info.height, map_gmapping.info.width);

    // The occupancy grid has been reshaped to be able to use C++ algorithms with it

    // Retrieving Resolution and Pose of the map in the world frame
    float resolution = map_gmapping.info.resolution;*/
    float x_pose = map_gmapping.info.origin.position.x;
    float y_pose = map_gmapping.info.origin.position.y;

    auto t_retrieve_map = std::chrono::high_resolution_clock::now();
    //////////////////////////////////////
    // PROCESSING POSITION OF THE ROBOT //
    //////////////////////////////////////

    // Get the transform between /map and /base_link (to get the position of the robot in the map)
    try
    {
      listener_.lookupTransform("map", "base_link", ros::Time(0), transform_);
      ROS_DEBUG("Transform is ready");

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
         ROS_DEBUG("Drift     | %f %f %f", drift_odometry_x, drift_odometry_y, drift_odometry_yaw);
      }

    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
    }
    
    // Get rotation information between "map" and "base_link"
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(transform_.getRotation());
    mat.getRPY(roll, pitch, yaw); // Assign values to roll pitch yaw variables

    //std::cout << transform_.getOrigin().getX() << " | " << x_pose << " | " << size_cell << std::endl; 
    //std::cout << transform_.getOrigin().getY() << " | " << y_pose << " | " << size_cell << std::endl; 
    
    // Create robot state vector and fill it
    state_robot << (transform_.getOrigin().getX() - x_pose)/ size_cell,
                   (transform_.getOrigin().getY() - y_pose)/ size_cell,
                   yaw;
    ROS_DEBUG("Robot     | %f %f %f", state_robot(0,0), state_robot(1,0), state_robot(2,0));


    auto t_process_robot = std::chrono::high_resolution_clock::now();
    //////////////////////////////////////////
    // PROCESSING POSITION OF THE ATTRACTOR //
    //////////////////////////////////////////

    // Get the (x,y) coordinates in the world frame of the cell (0,0)
    float start_cell_x = (-1 * std::round(map_gmapping.info.origin.position.x / size_cell));
    float start_cell_y = (-1 * std::round(map_gmapping.info.origin.position.y / size_cell));
    ROS_DEBUG("Start cell of attractor | %f %f", start_cell_x, start_cell_y);

    // Convert the position of the attractor into the occupancy map frame
    float position_goal_world_frame_corrected_x =  position_goal_world_frame_x * std::cos(-drift_odometry_yaw) + position_goal_world_frame_y * std::sin(-drift_odometry_yaw);
    float position_goal_world_frame_corrected_y =  position_goal_world_frame_x * (-1) * std::sin(-drift_odometry_yaw) + position_goal_world_frame_y * std::cos(-drift_odometry_yaw);
    //std::cout << position_goal_world_frame_corrected_x << " " << position_goal_world_frame_corrected_y << std::endl;
    float target_cell_x = start_cell_x + std::round((position_goal_world_frame_corrected_x+drift_odometry_x) / size_cell);
    float target_cell_y = start_cell_y + std::round((position_goal_world_frame_corrected_y+drift_odometry_y) / size_cell);
    //ROS_INFO("Starting cell %f %f",start_cell_x, start_cell_y);
    //ROS_INFO("Target cell %f %f",target_cell_x, target_cell_y);

    // Set state of the attractor
       State state_attractor;
       state_attractor << target_cell_x, target_cell_y, 0;
       ROS_DEBUG("Attractor | %f %f %f", target_cell_x, target_cell_y, 0.0);

    if (false)
    {
	if (static_cast<int>(key_command)==-1)
	{
		state_attractor << 0, 0, 0;
	}

	float scale = 3.0;
	float dir_x = 0.05; // Crash if 0 because state robot and state attractor are the same
	float dir_y = 0;
        float angle = yaw;
        ROS_INFO("Going in direction %i", static_cast<int>(key_command));
        switch(static_cast<int>(key_command))
	{
	 case 1: dir_x = -1.0; dir_y =  1.0; break;
	 case 2: dir_x = -1.0; dir_y =  0.0; break;
	 case 3: dir_x = -1.0; dir_y = -1.0; break;
	 case 4: dir_x =  0.0; dir_y =  1.0; break;
	 case 6: dir_x =  0.0; dir_y = -1.0; break;
	 case 7: dir_x =  1.0; dir_y =  1.0; break;
	 case 8: dir_x =  1.0; dir_y =  0.0; break;
	 case 9: dir_x =  1.0; dir_y = -1.0; break;
	}
	if ((static_cast<int>(key_command)!=0)&&(static_cast<int>(key_command)!=5)&&(static_cast<int>(key_command)!=-1))
	{
	state_attractor << (state_robot(0,0)+scale*dir_x*std::cos(angle)-scale*dir_y*std::sin(angle)),
			   (state_robot(1,0)+scale*dir_x*std::sin(angle)+scale*dir_y*std::cos(angle)),
	 		   0.0;
		key_command = 0;
        }
        ROS_INFO("Attractor | %f %f %f", state_attractor(0,0), state_attractor(1,0), 0.0);

    }

    geometry_msgs::PointStamped pt_attrac;
    pt_attrac.header.frame_id = "map";//map_gmapping.header;
    pt_attrac.header.stamp = ros::Time(0);
    pt_attrac.point.x = (state_attractor(0,0) + (x_pose / size_cell))*size_cell;
    pt_attrac.point.y = (state_attractor(1,0) + (y_pose / size_cell))*size_cell;
    pt_attrac.point.z = 0.05;
    ROS_DEBUG("Publishing /attractor");
    pub_attractor.publish(pt_attrac);

    float distance_to_attractor = std::sqrt(std::pow(state_robot(0,0)-state_attractor(0,0),2)+std::pow(state_robot(1,0)-state_attractor(1,0),2)); // Euclidian distance
    if (distance_to_attractor < 7)
    {
	num_attractor += 1;
    }
    if (num_attractor == max_num_attractor)
    {
	num_attractor = 0;
    }

    auto t_process_attractor = std::chrono::high_resolution_clock::now();
    /////////////////////////////////////
    // ADDING PEOPLE TO OCCUPANCY GRID //
    /////////////////////////////////////

    Eigen::MatrixXi eig_people;/* = eig_test;

    for (int i_col=0; i_col<detected_people.cols(); i_col++)
    {
        // Converting the position of the person into the occupancy map frame
        float person_cell_x = start_cell_x + std::round(detected_people(0,i_col) / size_cell);
        float person_cell_y = start_cell_y + std::round(detected_people(1,i_col) / size_cell);

        ROS_INFO("Drawing a circle at position (%f,%f)", person_cell_x, person_cell_y);

	// Drawing a disk centered on the position of the person in the occupancy grid
        draw_circle(eig_people, static_cast<int>(person_cell_x), static_cast<int>(person_cell_y), radius_in_cells);

        // Taking into account speed to estimate the position of the person 1 second in the future
        person_cell_x = start_cell_x + std::round((detected_people(0,i_col) + detected_people(2,i_col)*1.0) / size_cell);
        person_cell_y = start_cell_y + std::round((detected_people(1,i_col) + detected_people(3,i_col)*1.0) / size_cell);

        // Drawing a disk centered on the future position of the person in the occupancy grid
        draw_circle(eig_people, static_cast<int>(person_cell_x), static_cast<int>(person_cell_y), radius_in_cells);

        ROS_INFO("Drawing prediction at position (%f,%f)", person_cell_x, person_cell_y);
    }

    if (false) // enable to display a part of the occupancy map centered on the robot in the console
    {
        int corner_square_x = static_cast<int>(std::floor((transform_.getOrigin().getX() - x_pose)/size_cell)-25);
        int corner_square_y = static_cast<int>(std::floor((transform_.getOrigin().getY() - y_pose)/size_cell)-25);
        std::cout << eig_people.block( corner_square_x, corner_square_y, 51, 51) << std::endl;
    }*/

    auto t_adding_people = std::chrono::high_resolution_clock::now();
    ///////////////////////////////
    // PROCESSING OCCUPANCY GRID //
    ///////////////////////////////

    // Expand obstacles to get a security margin
    //Eigen::MatrixXi eig_expanded = expand_occupancy_grid( eig_people, n_expansion, state_robot, limit_in_cells, size_cell);
    //Eigen::MatrixXi eig_expanded_log = expand_occupancy_grid( eig_test, n_expansion, state_robot, limit_in_cells, size_cell);

    if (false) // enable to display a part of the occupancy map centered on the robot in the console
    {
        eig_expanded(state_robot(0,0),state_robot(1,0)) = -2;
        eig_expanded(state_attractor(0,0),state_attractor(1,0)) = -3;
        int corner_square_x = static_cast<int>(std::floor((transform_.getOrigin().getX() - x_pose)/size_cell)-25);
        int corner_square_y = static_cast<int>(std::floor((transform_.getOrigin().getY() - y_pose)/size_cell)-25);
        std::cout << eig_expanded.block( corner_square_x, corner_square_y, 51, 51) << std::endl;
    }

    if (false) // enable to save the expanded occupancy grid to "expanded_mapo.txt"
    {
	    std::ofstream mypoints;
	    mypoints.open("expanded_mapo.txt");
	    for (int i_row =0; i_row < eig_expanded.rows(); i_row++)
	    {
		for (int i_col =0; i_col < eig_expanded.cols(); i_col++)
		{
		    mypoints << eig_expanded(i_row, i_col);
		    if ((i_col+1) < eig_expanded.cols())
		    {
		        mypoints << " , ";
		    }
		}
		mypoints << "\n";
	    }
	    mypoints.close();
    }



    auto t_process_grid = std::chrono::high_resolution_clock::now();
    /////////////////////////
    // DETECTING OBSTACLES //
    /////////////////////////

    /*Grid grid_tempo;
    if (logging_enabled)
    {
        grid_tempo = eig_test;
    }*/

    // Detect expanded obstacles
    //std::vector<Border> storage;
    // storage = detect_borders( eig_expanded, state_robot );

    if (logging_enabled)
    {
        // Storing all occupied cells of the occupancy grid
        /*std::vector<Blob> storage_blobs;
	    storage_blobs = detect_blobs( grid_tempo );
	    for (int i=0; i<storage_blobs.size(); i++)
	    {
	        int n_rows = log_matrix.rows();
	        log_matrix.conservativeResize(n_rows+storage_blobs[i].rows(), Eigen::NoChange);
	        log_matrix.block(n_rows,0,storage_blobs[i].rows(),1) = Eigen::MatrixXf::Zero(storage_blobs[i].rows(),1);
	        log_matrix.block(n_rows,1,storage_blobs[i].rows(),1) = Eigen::MatrixXf::Ones(storage_blobs[i].rows(),1);
	        log_matrix.block(n_rows,2,storage_blobs[i].rows(),2) = ((storage_blobs[i]).block(0,0,storage_blobs[i].rows(),2)).template cast<float>();
	        log_matrix.block(n_rows,4,storage_blobs[i].rows(),3) = Eigen::MatrixXf::Zero(storage_blobs[i].rows(),3);
	    }*/

	    // Storing position of the robot in the initial space (feature 5)
        log_matrix.conservativeResize(log_matrix.rows()+1, Eigen::NoChange);
        log_matrix.row(log_matrix.rows()-1) << 0.0, 5.0, state_robot(0,0), state_robot(1,0), 0.0,0.0,0.0;

        // Storing position of the robot in the initial space (feature 6)
        log_matrix.conservativeResize(log_matrix.rows()+1, Eigen::NoChange);
        log_matrix.row(log_matrix.rows()-1) << 0.0, 6.0, state_attractor(0,0), state_attractor(1,0), 0.0,0.0,0.0;

    }

    if (false) // enable to display detected borders
    {
    	for (int iter=0; iter < storage.size(); iter++)
    	{
        	std::cout << "Border " << iter << ":"<< std::endl;
       		std::cout << storage[iter] << std::endl;
    	}
    }

    if (false) // enable to save the occupied cells of detected obstacles to text files
    {
	// Detecting blobs
	    std::vector<Blob> storage_blobs;
	    storage_blobs = detect_blobs( eig_expanded );
	// Opening text file
	    std::ofstream myblobs;
	    std::cout << storage_blobs.size() << " blobs have been detected." << std::endl;

	    for (int iter=0; iter < storage_blobs.size(); iter++)
	    {
		std::cout << "Iterator: " << iter << std::endl;
		myblobs.open("./gazebo_obstacle_" + std::to_string(iter) + ".txt"); // Each obstacle has its own file
		std::cout << "Blob " << iter << " opened." << std::endl;
		Blob blob = storage_blobs[iter];
		for (int k=0; k < blob.rows(); k++)
		{
		     myblobs << "obst" << iter << ".row(" << k << ") << " << blob(k,0) << " , " << blob(k,1) << "; \n";
		}
		myblobs.close();
	    }
    }

    if (false) // enable to save the detected borders to text files
    {
	    std::ofstream myobstacles;
	    std::cout << storage.size() << " obstacles have been detected." << std::endl;
	    int size_storage = storage.size();
	    for (int iter=0; iter < storage.size(); iter++)
	    {
		std::cout << "Iterator: " << iter << std::endl;
		myobstacles.open("./gazebo_obstacle_debug" + std::to_string(size_storage) + "_" + std::to_string(iter) + ".txt"); // Each border has its own file
		std::cout << "Obstacle " << iter << " opened." << std::endl;
		Border blob = storage[iter];
		for (int k=0; k < blob.rows(); k++)
		{
		     myobstacles << "obst" << iter << ".row(" << k << ") << " << blob(k,0) << " , " << blob(k,1) << "; \n";
		}
		myobstacles.close();
	    }
    }


    auto t_detect_obs = std::chrono::high_resolution_clock::now();
    ///////////////////////////////////
    // COMPUTE NEXT VELOCITY COMMAND //
    ///////////////////////////////////

    // Compute velocity command based on the detected obstacles (old version, no limit distance)
    // State next_eps = next_step_several_obstacles_border( state_robot, state_attractor, storage);

    // Compute velocity command based on the detected obstacles (new version, only closest obstacle)
    /*Eigen::Matrix<float, 1, 2> robot; robot << state_robot(0,0), state_robot(1,0);
    int closest_i = 0;
    int closest_dist = gamma_norm_proj(0,0);

    for (int iter=1; iter< storage.size(); iter++)
    {
         closest = find_closest_point(robot, storage[iter]);
         gamma_norm_proj = gamma_normal_projection( robot, closest);
         if (gamma_norm_proj(0,0) < closest_dist)
         {
              closest_i = iter;
              closest_dist = gamma_norm_proj(0,0);
         }
    }
    State next_eps = next_step_special( state_robot, state_attractor, storage[closest_i]); */

    // Compute velocity command based on the detected obstacles (new version, all obstacles within limit range)
    State next_eps = test_next_step_special_weighted( state_robot, state_attractor, storage, size_cell);
    ROS_DEBUG("VelCmd in map frame: %f %f %f", next_eps(0,0), next_eps(1,0), next_eps(2,0));

    auto t_compute_vel = std::chrono::high_resolution_clock::now();


    /////////////////////////////////////////
    // COMPUTING DATA TO GET A STREAM PLOT //
    /////////////////////////////////////////
        if (false)
        {
            if (std::fmod(static_cast<float>(ID_frame),10) == 0)
            {


            // Declare file for recording
            std::ofstream mystream;
            if (ID_frame < 10)
            {
                mystream.open("./StreamNode/eigpeople_000" + std::to_string(ID_frame) + ".txt");
            }
            else if (ID_frame < 100)
            {
                mystream.open("./StreamNode/eigpeople_00" + std::to_string(ID_frame) + ".txt");
            }
            else if (ID_frame < 1000)
            {
                mystream.open("./StreamNode/eigpeople_0" + std::to_string(ID_frame) + ".txt");
            }
            else
            {
                mystream.open("./StreamNode/eigpeople_" + std::to_string(ID_frame) + ".txt");
            }


            // Expand all occupied cells of the occupancy grid by n cells
            Grid occupancy_res = eig_people;
            for (int n=0; n<n_expansion; n++)
            {
                Grid occupancy_temp = occupancy_res; // occupancy grid at step n
                for (int i=1; i<(occupancy_res.rows()-1); i++) // scan row i
                {
                   for (int j=1; j<(occupancy_res.cols()-1); j++) // scan col j
                   {
                       if (occupancy_temp(i,j) == 100) // if occupied we fill the 4 neightbour cells
                       {
                           occupancy_res(i+1,j) = 100;
                           occupancy_res(i-1,j) = 100;
                           occupancy_res(i,j+1) = 100;
                           occupancy_res(i,j-1) = 100;
                       }
                   }
                }
            }

            mystream << occupancy_res << "\n";
            mystream.close();

            // Detecting blobs
            std::vector<Blob> storage_blobs;
            storage_blobs = detect_blobs( occupancy_res );

            // Opening text file
            std::ofstream myblobs;

            // Declare file for recording
            if (ID_frame < 10)
            {
                myblobs.open("./StreamNode/obstacles_000" + std::to_string(ID_frame) + ".txt");
            }
            else if (ID_frame < 100)
            {
                myblobs.open("./StreamNode/obstacles_00" + std::to_string(ID_frame) + ".txt");
            }
            else if (ID_frame < 1000)
            {
                myblobs.open("./StreamNode/obstacles_0" + std::to_string(ID_frame) + ".txt");
            }
            else
            {
                myblobs.open("./StreamNode/obstacles_" + std::to_string(ID_frame) + ".txt");
            }

            for (int iter=0; iter < storage_blobs.size(); iter++)
            {
                Blob blob = storage_blobs[iter];
                /*for (int k=0; k < blob.rows(); k++)
                {
                    myblobs << blob(k,0) << " , " << blob(k,1) << "\n";
                }*/
                myblobs << blob << "\n";

            }
            myblobs.close();
            }
            ID_frame+=1;
        }

    if (false)
    {

        // Declare file for recording
        std::ofstream mystream;
        if (ID_frame < 10)
        {
            mystream.open("./StreamNode/stream_data_border_00" + std::to_string(ID_frame) + ".txt");
        }
        else if (ID_frame < 100)
        {
            mystream.open("./StreamNode/stream_data_border_0" + std::to_string(ID_frame) + ".txt");
        }
        else if (ID_frame < 100)
        {
            mystream.open("./StreamNode/stream_data_border_" + std::to_string(ID_frame) + ".txt");
        }
        ID_frame+=1;

        // Limits of stream
        Eigen::Matrix<float, 5, 1> limits;
        limits << -2.02, 17.02, -2.02, 17.02, 0.2;

        for (float x=limits(0,0); x <= limits(1,0); x += limits(4,0)) // x direction of the grid
        {
            std::cout << x << std::endl;
            for (float y=limits(2,0); y <= limits(3,0); y += limits(4,0)) // y direction of the grid
            {
                float temp_cell_x = start_cell_x + std::round(x / size_cell);
                float temp_cell_y = start_cell_y + std::round(y / size_cell);

                // Position of the point
                State state_point;
                state_point << temp_cell_x, temp_cell_y, 0;

                // Expand occupancy grid (obstacles in range)
                eig_expanded = expand_occupancy_grid( eig_people, n_expansion, state_point, limit_in_cells, size_cell);

                // Detect borders of obstacles in range
                storage = detect_borders( eig_expanded, state_point );

                // Compute velocity command
                next_eps = next_step_special_weighted( state_point, state_attractor, storage, size_cell);

                mystream << x << "," << y << "," << next_eps(0,0) << "," << next_eps(1,0) << "\n"; // write result in text file
            }
        }

        mystream.close();
    }

    //////////////////////////////////////
    // PROCESSING NEXT VELOCITY COMMAND //
    //////////////////////////////////////

    // We have to convert the velocity command from the map frame to the base_link frame
    // because the Ridgeback uses a velocity command in its own frame

    tf::Vector3 vec3; // Need to use the type tf::Vector3 to make the convertion
    vec3.setX(next_eps(0,0));
    vec3.setY(next_eps(1,0));
    vec3.setZ(0);
    tf::Vector3 vec3_transformed;

    tf::Stamped<tf::Vector3> vec3_stamped(vec3, ros::Time(0), "map");

    tf::Stamped<tf::Vector3> vec3_stamped_transformed(vec3_transformed, ros::Time(0), "base_link");

    if ( listener_.canTransform("base_link", "map", ros::Time(0)))
    {
            listener_.transformVector( "base_link", vec3_stamped, vec3_stamped_transformed);

	    ROS_DEBUG("VelCmd in base_link frame: %f %f %f", vec3_stamped_transformed.x(), vec3_stamped_transformed.y(), next_eps(2,0));

	    // Creating velocity message
	    geometry_msgs::Twist output;
	    output.linear.x =  vec3_stamped_transformed.x();
	    output.linear.y =  vec3_stamped_transformed.y();
	    output.linear.z =  0.0;
	    output.angular.x = 0.0;
	    output.angular.y = 0.0;
	    output.angular.z = next_eps(2,0);
 
            // Check if there is a NaN value in the velocity command
            // Suprisingly the Ridgeback is not protected against that and just goes to its max speed...
            if ((std::isnan(output.linear.x))||(std::isnan(output.linear.y))||(std::isnan(output.angular.z)))
            {
		 output.linear.x = 0.0; output.linear.y = 0.0; output.angular.z = 0.0;
                 ROS_WARN("NaN value detected in the final velocity command.");
                 ROS_DEBUG("VelCmd in base_link frame: %f %f %f", output.linear.x, output.linear.y, output.angular.z);
            }
	    //ROS_INFO("VelCmd in Ridgeback frame: %f %f %f", next_eps(0,0), next_eps(1,0), next_eps(2,0));
	    //if ((static_cast<int>(key_command)!=5)&&(static_cast<int>(key_command)!=-1)) // if we press 5, the robot should not move
	   // {
	        pub_.publish(output);
	    //}
            

    }
    else
    {
            ROS_ERROR("Cannot tramsform velocity vector from map to base_link");
    }

    auto t_send_vel = std::chrono::high_resolution_clock::now();

    ///////////////////////////////
    // SAVING data_obstacles.txt //
    ///////////////////////////////
    float timestamp = 0.0;
    if (logging_enabled)
    {
	//std::cout << ros::Time::now().toSec() << " VS " << time_start_clock << std::endl;
        timestamp = static_cast<float>(ros::Time::now().toSec() - time_start_clock);
	Eigen::MatrixXf time_matrix = timestamp * Eigen::MatrixXf::Ones(log_matrix.rows(),1);
        Eigen::MatrixXf matrix(log_matrix.rows(), 1+log_matrix.cols());
        matrix << time_matrix, log_matrix;
	//std::cout << log_matrix << std::endl;

	const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
    mylog << matrix.format(CSVFormat) << "\n";
    log_matrix = Eigen::MatrixXf::Zero(1,7);
	//mylog << matrix << "\n";

        std_msgs::Float32 clock_msg;
        clock_msg.data = timestamp;
        pub_clock.publish(clock_msg);

    }


    //////////////////////////
    // END OF MAIN CALLBACK //
    //////////////////////////

     float timestamp_timing = timestamp;//ros::Time::now().toSec();
    if (true)
    {
     std::chrono::duration<double> diff = t_parameters - t_start;
     //std::cout << "T parameters:        " << diff.count() << std::endl;
     my_timing << 0 << "," << timestamp_timing << "," << diff.count() << "\n";
     diff = t_retrieve_map - t_parameters;
     //std::cout << "T retrieve map:      " << diff.count() << std::endl;
     my_timing << 1 << "," << timestamp_timing << "," << diff.count() << "\n";
     diff = t_process_robot - t_retrieve_map;
     //std::cout << "T process robot:     " << diff.count() << std::endl;
     my_timing << 2 << "," << timestamp_timing << "," << diff.count() << "\n";
     diff = t_process_attractor - t_process_robot;
     //std::cout << "T process attractor: " << diff.count() << std::endl;
     my_timing << 3 << "," << timestamp_timing << "," << diff.count() << "\n";
     /*diff = t_adding_people - t_process_attractor;
     std::cout << "T adding people:     " << diff.count() << std::endl;
     my_timing << 4 << "," << timestamp_timing << "," << diff.count() << "\n";*/
     /*diff = t_process_grid - t_adding_people;
     std::cout << "T process grid:      " << diff.count() << std::endl;
     my_timing << 5 << "," << timestamp_timing << "," << diff.count() << "\n";
     diff = t_detect_obs - t_process_grid;
     std::cout << "T detect obstacles:  " << diff.count() << std::endl;
     my_timing << 6 << "," << timestamp_timing << "," << diff.count() << "\n";*/
     diff = t_compute_vel - t_detect_obs;
     //std::cout << "T compute cmd_vel:   " << diff.count() << std::endl;
     my_timing << 7 << "," << timestamp_timing << "," << diff.count() << "\n";
     diff = t_send_vel - t_compute_vel;
     //std::cout << "T send cmd_vel:      " << diff.count() << std::endl;
     my_timing << 8 << "," << timestamp_timing << "," << diff.count() << "\n";

     diff = t_send_vel - t_start;
     //std::cout << "T all loop:          " << diff.count() << std::endl;
    }

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

void callback_for_key(const geometry_msgs::Twist& input) // Callback triggered by /cmd_key topic
{
	key_command = input.linear.x;
}

void callback_for_map(const nav_msgs::OccupancyGrid& input) // Callback triggered by /map topic
{
    float timestamp_timing = ros::Time::now().toSec();

    auto t_start = std::chrono::high_resolution_clock::now();

    std::cout << " == Map received from Gmapping node == " << std::endl;
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

    ////////////////////////////////////////////////////
    // PROCESSING POSITION OF THE ROBOT ONCE FOR INIT //
    ////////////////////////////////////////////////////

    if (state_robot(2,0)==(-42))
    {
        // Get the transform between /map and /base_link (to get the position of the robot in the map)
        bool has_map = false;
        do {
                has_map = false;
                try
                {
                  listener_.lookupTransform("map", "base_link", ros::Time(0), transform_);
                  ROS_INFO("Transform is ready");
                }
                catch (tf::TransformException &ex)
                {
                    ROS_ERROR("%s",ex.what());
                    has_map = true;
                }
        } while (has_map);

        // Get rotation information between "map" and "base_link"
        tfScalar yaw, pitch, roll;
        tf::Matrix3x3 mat(transform_.getRotation());
        mat.getRPY(roll, pitch, yaw); // Assign values to roll pitch yaw variables

        // Create robot state vector and fill it
        state_robot << (transform_.getOrigin().getX() - x_pose)/ size_cell,
                                          (transform_.getOrigin().getY() - y_pose)/ size_cell,
                                          yaw;
        ROS_INFO("Robot     | %f %f %f", state_robot(0,0), state_robot(1,0), state_robot(2,0));
    }
    ///////////////////////////////
    // PROCESSING OCCUPANCY GRID //
    ///////////////////////////////

    // Expand obstacles to get a security margin
    eig_expanded = expand_occupancy_grid( eig_test, n_expansion, state_robot, limit_in_cells, size_cell);

    if (false) // enable to display a part of the occupancy map centered on the robot in the console
    {
        eig_expanded(state_robot(0,0),state_robot(1,0)) = -2;
        //eig_expanded(state_attractor(0,0),state_attractor(1,0)) = -2;
        int corner_square_x = static_cast<int>(std::floor((transform_.getOrigin().getX() - x_pose)/size_cell)-25);
        int corner_square_y = static_cast<int>(std::floor((transform_.getOrigin().getY() - y_pose)/size_cell)-25);
        std::cout << eig_expanded.block( corner_square_x, corner_square_y, 51, 51) << std::endl;
    }

    auto t_process_grid = std::chrono::high_resolution_clock::now();

	/////////////////////////
    // DETECTING OBSTACLES //
    /////////////////////////

    // Detect expanded obstacles
    storage = detect_borders( eig_expanded, state_robot );

    auto t_detect_obs = std::chrono::high_resolution_clock::now();

    if (true)
    {
         std::chrono::duration<double> diff = t_process_grid - t_start;
         std::cout << "T process grid:      " << diff.count() << std::endl;
         my_timing << 5 << "," << timestamp_timing << "," << diff.count() << "\n";
         diff = t_detect_obs - t_process_grid;
         std::cout << "T detect obstacles:  " << diff.count() << std::endl;
         my_timing << 6 << "," << timestamp_timing << "," << diff.count() << "\n";
    }
}


void callback_for_boundary(const std_msgs::Float32MultiArray& input) // Callback triggered by /boundary_cells topic
{
	int max_obs = static_cast<int>(input.layout.dim[0].size);
        int max_row = static_cast<int>(input.layout.dim[1].size);
        
        // Erase old content of storage and fill it with the new content
        storage.clear();
	for (int i=0; i<max_obs; i++)
        {
	    Border boundary;

            /*int num_of_rows = 0;
            float charac1, charac2;
            do { 
                charac1 = input.data[static_cast<int>(input.layout.data_offset + max_row*i + 5*num_of_rows + 3)];
                charac2 = input.data[static_cast<int>(input.layout.data_offset + max_row*i + 5*num_of_rows + 4)];
                num_of_rows += 1;
            } while (((charac1!=0)||(charac2!=0))&&(num_of_rows<=max_row));
            num_of_rows -= 1;*/

            int num_of_rows = 0;
            float charac;
            do { 
                charac = input.data[static_cast<int>(input.layout.data_offset + max_row*i*5 + 5*num_of_rows + 2)];
                num_of_rows += 1;
            } while ((charac!=0)&&(num_of_rows<=max_row));
            num_of_rows -= 1;

            //std::cout << "Boundary " << i << " has " << num_of_rows << " cells." << std::endl;

            boundary = Eigen::MatrixXf::Zero(num_of_rows,5);
            //std::cout << static_cast<int>(input.layout.data_offset + max_row*i*5 + 0 + 0) << std::endl;
	    for (int j=0; j<num_of_rows; j++)
            {
                    boundary.row(j) << 
                    input.data[static_cast<int>(input.layout.data_offset + max_row*i*5 + 5*j + 0)],
                    input.data[static_cast<int>(input.layout.data_offset + max_row*i*5 + 5*j + 1)],
                    input.data[static_cast<int>(input.layout.data_offset + max_row*i*5 + 5*j + 2)],
                    input.data[static_cast<int>(input.layout.data_offset + max_row*i*5 + 5*j + 3)],
                    input.data[static_cast<int>(input.layout.data_offset + max_row*i*5 + 5*j + 4)];
            }
            storage.push_back(boundary);
        }

        // storage.clear(); // NO OBSTACLE IN RANGE

        /*std::cout << "Displaying result" << std::endl;
        for (int i=0; i<storage.size(); i++)
        {
            //std::cout << "Row 0: " << (storage[i]).row(0) << std::endl;
            std::cout << "Obstacle " << i << std::endl;
            if ((storage[i]).rows()>5) {std::cout << (storage[i]).block(0,0,5,5) << std::endl;}
            else {std::cout << (storage[i]).row(0) << std::endl;}
        }*/
         /*for (int i=0; i<storage.size(); i++)
        {
          Eigen::MatrixXf coeff = Eigen::MatrixXf::Zero(1,1);
             
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
            std::cout << temporaire << std::endl;}*/

}

void callback_for_map_info(const nav_msgs::OccupancyGrid& input) // Callback triggered by /map_info topic
{
	map_gmapping = input;
}

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;  // To publish the velocity command
  ros::Publisher pub_attractor;  // To publish the position of the attractor (for display)
  ros::Publisher pub_clock;  // To publish logging clock for log_data_robot_node
  ros::Subscriber sub_; // To listen to the trigger topic
  ros::Subscriber sub_people; // To listen to the pose_people topic
  ros::Subscriber sub_key;    // To listen to the cmd_key topic
  ros::Subscriber sub_map;    // To listen to the map topic
  ros::Subscriber sub_boundary; // To listen to the boundary_cells topic
  ros::Subscriber sub_map_info; // To listen to map_info topic
  tf::TransformListener listener_; // To listen to transforms
  tf::StampedTransform transform_; // Transform from map to base_link
  ros::ServiceClient client_map_;  // To call dynamic_map service of gmapping
  Eigen::MatrixXf detected_people;
  float key_command; // between 1 and 9 for direction
  int num_attractor; // numero of the attractor that is currently being followed

  int ID_frame; // for stream recording
  float time_start;
  double time_start_clock; // to know when the node has been launched
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

  bool init_attractor;
  float drift_odometry_x;
  float drift_odometry_y;
  float drift_odometry_yaw;
};//End of class SubscribeAndPublish



int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "process_occupancy_grid_node");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  // Spin node
  ros::spin();

  return 0;
}


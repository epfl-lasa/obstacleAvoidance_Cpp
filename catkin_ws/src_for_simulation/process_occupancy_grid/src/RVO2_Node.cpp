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

//#include "obstacle_avoidance/GetObstacles.h"
#include <cstdlib>

// Eigen library and the header that contains my functions
#include <eigen3/Eigen/Core>
#include "ObstacleReconstruction.h"
#include "ObstacleAvoidance.h"
#include "RVO/RVO.h"
#include "RVO/RVOSimulator.h"

#include <fstream>  // To write data into files

#include <cstddef>
#include <cmath>
#include <ctime>


#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif

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

    //Topic you want to subscribe (trigger signal)
    sub_ = n_.subscribe("/test", 1, &SubscribeAndPublish::callback, this);

    //Another topic you want to subscribe (position of detected people)
    sub_people = n_.subscribe("/pose_people_map_disabled", 2, &SubscribeAndPublish::callback_for_people, this);

    //Another topic you want to subscribe (attractor control with keyboard)
    sub_key = n_.subscribe("/cmd_key", 2, &SubscribeAndPublish::callback_for_key, this);

    // Client of dynamic_map service of gmapping (to get the occupancy grid)
    client_map_ = n_.serviceClient<nav_msgs::GetMap>("dynamic_map");

    // Transform listener
    tf::TransformListener transform_(ros::Duration(10.0));

    // Default command
    key_command = -1;

    // Default attractor
    num_attractor = 0;

    //Topic to publish people's position for Gazebo
    pub_RVO = n_.advertise<geometry_msgs::PoseArray>("/poses_for_gazebo", 10);

  }

  void callback(const geometry_msgs::Twist& input) // callback is triggered by /test topic
  {
    ////////////////
    // PARAMETERS //
    ////////////////

    // Size of gmapping cells (the one you use for delta in rosrun gmapping slam_gmapping scan:=/scan _delta:=0.3 _map_update_interval:=1.0)
    float size_cell = 0.15;

    // Position of the attractor compared to the initial position of the robot (in meters)
    // Consider that the robot starts at position (0,0)
    // If you set the attractor at (12,-11) the target is 12m forward and 11m to the right of the starting position
    //float position_goal_world_frame_x =  12;// 2.52; //  42; // 23.6; //  15;
    //float position_goal_world_frame_y = -11;//-2.56;//-7.3; //5   ; // 0;

    int max_num_attractor = 4;
    float position_goal_world_frame_x =  30;
    float position_goal_world_frame_y =  0;
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


    // Radius of the Ridgeback
    float radius_ridgeback = 0.4;

    // Number of cells that the obstacles should be expanded
    int n_expansion = static_cast<int>(std::ceil(radius_ridgeback/size_cell));

    // Limit distance to consider obstacles (in meters)
    float limit_in_meters = 3;
    int   limit_in_cells  = static_cast<int>(std::ceil(limit_in_meters/size_cell));

    // Radius of the security circle around people (in meters)
    // It does not include the radius of the ridgeback ~0.6 meters (disk will be expanded)
    float radius_around_people = 0.3;
    int   radius_in_cells = static_cast<int>(std::ceil(radius_around_people / size_cell)); // radius_around_people is in [m] and we need a value in [cell]

    //////////////////////////////////
    // RETRIEVING MAP FROM GMAPPING //
    //////////////////////////////////

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
    // ROS_INFO("Map origin: %f %f", x_pose, y_pose);

    // NO NEED SINCE THE MAP IS USED AS THE REFERENCE FRAME
    /*tfScalar roll, pitch, yaw;
    tf::Matrix3x3 mat(srv_map.response.map.info.origin.orientation);
    mat.getRPY(roll, pitch, yaw);
    float phi_pose = yaw;*/

    //////////////////////////////////////
    // PROCESSING POSITION OF THE ROBOT //
    //////////////////////////////////////

    // Get the transform between /map and /base_link (to get the position of the robot in the map)
    try
    {
      listener_.lookupTransform("map", "base_link", ros::Time(0), transform_);
      ROS_INFO("Transform is ready");
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s",ex.what());
    }

    // Get rotation information between "map" and "base_link"
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(transform_.getRotation());
    mat.getRPY(roll, pitch, yaw); // Assign values to roll pitch yaw variables

    // Create robot state vector and fill it
    State state_robot; state_robot << (transform_.getOrigin().getX() - x_pose)/ size_cell,
                                      (transform_.getOrigin().getY() - y_pose)/ size_cell,
                                      yaw;
    ROS_INFO("Robot     | %f %f %f", state_robot(0,0), state_robot(1,0), state_robot(2,0));


    //////////////////////////////////////////
    // PROCESSING POSITION OF THE ATTRACTOR //
    //////////////////////////////////////////

    // Get the (x,y) coordinates in the world frame of the cell (0,0)
    float start_cell_x = (-1 * std::round(srv_map.response.map.info.origin.position.x / size_cell));
    float start_cell_y = (-1 * std::round(srv_map.response.map.info.origin.position.y / size_cell));

    // Convert the position of the attractor into the occupancy map frame
    float target_cell_x = start_cell_x + std::round(position_goal_world_frame_x / size_cell);
    float target_cell_y = start_cell_y + std::round(position_goal_world_frame_y / size_cell);
    //ROS_INFO("Starting cell %f %f",start_cell_x, start_cell_y);
    //ROS_INFO("Target cell %f %f",target_cell_x, target_cell_y);

    // Set state of the attractor
       State state_attractor;
       state_attractor << target_cell_x, target_cell_y, 0;
       ROS_INFO("Attractor | %f %f %f", target_cell_x, target_cell_y, 0.0);

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
    pt_attrac.header = srv_map.response.map.header;
    pt_attrac.point.x = (state_attractor(0,0) + (x_pose / size_cell))*size_cell;
    pt_attrac.point.y = (state_attractor(1,0) + (y_pose / size_cell))*size_cell;
    pt_attrac.point.z = 0.05;
    ROS_INFO("Publishing /attractor");
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

    /////////////////////////////////////
    // ADDING PEOPLE TO OCCUPANCY GRID //
    /////////////////////////////////////

    Eigen::MatrixXi eig_people = eig_test;
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
    }

    ///////////////////////////////
    // PROCESSING OCCUPANCY GRID //
    ///////////////////////////////

    // Expand obstacles to get a security margin
    Eigen::MatrixXi eig_expanded = expand_occupancy_grid( eig_people, n_expansion, state_robot, limit_in_cells, size_cell);

    if (false) // enable to display a part of the occupancy map centered on the robot in the console
    {
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


    /////////////////////////
    // DETECTING OBSTACLES //
    /////////////////////////

    // Detect expanded obstacles
    std::vector<Border> storage;
    storage = detect_borders( eig_expanded, state_robot );

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




    ///////////////////////////////////////
    // INITIALIZATION OF RVO2 simulation //
    ///////////////////////////////////////

    /* Velocity command */
    State next_eps;

    if (detected_people.cols()==0) /* First initialization step */
    {
        ROS_INFO("Detected_people is empty, entering initialization");

        init_detected_people(); // TODO: Not really needed?

        /*geometry_msgs::PoseArray people_RVO;
        people_RVO.header.stamp = ros::Time(0);
        people_RVO.header.frame_id = "map";

        // Pose entity
        geometry_msgs::Pose person_RVO;

        // Person's position
        person_RVO.position.x = 0.0;
        person_RVO.position.y = 0.0;
        person_RVO.position.z = -42.0;

        // Person's orientation
        person_RVO.orientation.x = 0.0;
        person_RVO.orientation.y = 0.0;
        person_RVO.orientation.z = 0.0;
        person_RVO.orientation.w = 1.0;

        // Add person to the array of poses
        people_RVO.poses.push_back(person_RVO);

        // Publish for Gazebo
        pub_RVO.publish(people_RVO);

        // Apply null velocity command
        next_eps << 0.0, 0.0, 0.0;

        ROS_INFO("Sending initialization array to Gazebo");

        // Publish for Gazebo
        pub_RVO.publish(people_RVO);*/


        geometry_msgs::PoseArray people_RVO;
        people_RVO.header.stamp = ros::Time(0);
        people_RVO.header.frame_id = "map";

        for (int i_col=0; i_col<detected_people.cols(); i_col++)
        {
            /* Agent 0 is the Ridgeback so starting with an offset of 1 */


            geometry_msgs::Pose person_RVO;

            /* Person's position */
            person_RVO.position.x = detected_people(0,i_col);
            person_RVO.position.y = detected_people(1,i_col);
            person_RVO.position.z = i_col;

            /* Person's orientation */
            // TODO: Change quaternion to align people with the direction they are going
            person_RVO.orientation.x = 0.0;
            person_RVO.orientation.y = 0.0;
            person_RVO.orientation.z = 0.0;
            person_RVO.orientation.w = 1.0;

            /* Add person to the array of poses */
            people_RVO.poses.push_back(person_RVO);

            // TODO: Update detected_people with new position and speed
            // Speed to 0???
            detected_people.col(i_col) << person_RVO.position.x, person_RVO.position.y, 0, 0, 0, detected_people(5,i_col), detected_people(6,i_col), 0, 0, 0;

        }

        /* Publish for Gazebo */
        pub_RVO.publish(people_RVO);



    }
    else /* Call simulation to get next position of the people */
    {
        /* Create a new simulator instance. */
        RVO::RVOSimulator *sim = new RVO::RVOSimulator();

        /* Specify the global time step of the simulation. */
        sim->setTimeStep(0.25f);

        /* Specify the default parameters for agents that are subsequently added. */
        sim->setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 0.5f, 0.5f);

        /* Add Ridgeback platform */
        sim->addAgent(RVO::Vector2( state_robot(0,0), state_robot(1,0)));
        goals.push_back(RVO::Vector2(target_cell_x, target_cell_y));

        /*
         * Add agents, specifying their start position, and store their goals on the
         * opposite side of the environment.
         */
        std::cout << "Adding agents" << std::endl;
        for (int i_col=0; i_col<detected_people.cols(); i_col++)
        {
            // A person: [x_c, y_c, v_x, v_y, stamp, x_goal, y_goal, 0, 0, 0]

            // Converting the position of the person into the occupancy map frame
            float person_cell_x = start_cell_x + std::round(detected_people(0,i_col) / size_cell);
            float person_cell_y = start_cell_y + std::round(detected_people(1,i_col) / size_cell);

            float person_goal_cell_x = start_cell_x + std::round(detected_people(5,i_col) / size_cell);
            float person_goal_cell_y = start_cell_y + std::round(detected_people(6,i_col) / size_cell);

            sim->addAgent(RVO::Vector2(    person_cell_x, person_cell_y));
            goals.push_back(RVO::Vector2( person_goal_cell_x,person_goal_cell_y));

            std::cout << "Agent " << 1 + i_col << " at position (" << person_cell_x << "," << person_cell_y << ")" << std::endl;
            std::cout << "In real world at position (" << detected_people(0,i_col) << "," << detected_people(1,i_col) << ")" << std::endl;
            std::cout << "With goal (" << person_goal_cell_x << "," << person_goal_cell_y << ")" << std::endl;
            std::cout << "In real world at position (" << detected_people(5,i_col) << "," << detected_people(6,i_col) << ")" << std::endl;

        }


        /* Convert each obstacle into vertices */
        ROS_INFO( "Converting obstacles into vertices" );
        for (int iter=0; iter < storage.size(); iter++)
        {

            std::vector<RVO::Vector2> obstacle;
            Border blob = storage[iter];
            if (iter==0) {std::cout << blob << std::endl;}
            //for (int k=(blob.rows()-1); k >=0; k--)
            for (int k=0; k < blob.rows(); k++)
            {

                switch (static_cast<int>(blob(k,2)))
                {
                case 1: // Straight lines
                    if (blob(k,3)==1) // normal vector along X axis
                    {
                        obstacle.push_back(RVO::Vector2( blob(k,0),  blob(k,1)+(1*0.5)));
                    }
                    else if (blob(k,3)==(-1))
                    {
                        obstacle.push_back(RVO::Vector2( blob(k,0),  blob(k,1)-(1*0.5)));
                    }
                    else if (blob(k,4)==(+1))
                    {
                        obstacle.push_back(RVO::Vector2( blob(k,0)-(1*0.5),  blob(k,1)));
                    }
                    else if (blob(k,4)==(-1))
                    {
                        obstacle.push_back(RVO::Vector2( blob(k,0)+(1*0.5),  blob(k,1)));
                    }
                    else
                    {
                        throw std::invalid_argument("Should not happen. Invalid straight line cell." );
                    }
                    break;
                case 2: // Outward circles
                    switch (static_cast<int>(blob(k,4)))
                    {
                    case 0: obstacle.push_back(RVO::Vector2( blob(k,0)-(1*0.5),  blob(k,1))); break;
                    case 1: obstacle.push_back(RVO::Vector2( blob(k,0),  blob(k,1)-(1*0.5))); break;
                    case 2: obstacle.push_back(RVO::Vector2( blob(k,0)+(1*0.5),  blob(k,1))); break;
                    case 3: obstacle.push_back(RVO::Vector2( blob(k,0),  blob(k,1)+(1*0.5))); break;
                    }
                    break;
                case 3: // Inward circles
                    switch (static_cast<int>(blob(k,4)))
                    {
                    case 0: obstacle.push_back(RVO::Vector2( blob(k,0),  blob(k,1)-(1*0.5))); break;
                    case 1: obstacle.push_back(RVO::Vector2( blob(k,0)+(1*0.5),  blob(k,1))); break;
                    case 2: obstacle.push_back(RVO::Vector2( blob(k,0),  blob(k,1)+(1*0.5))); break;
                    case 3: obstacle.push_back(RVO::Vector2( blob(k,0)-(1*0.5),  blob(k,1))); break;
                    }
                    break;
                default: throw std::invalid_argument("Should not happen. Closest border cell has no type." ); break;
                }
            }
            if (iter==0)
            {
                for (int n = 0; n < obstacle.size(); n++)
                {
                    std::cout << "Vertice: " << (obstacle[n]).x() << " " << (obstacle[n]).y() << std::endl;
                }

            }
            /*
            * Add (polygonal) obstacles, specifying their vertices in counterclockwise
            * order.
            */
            //sim->addObstacle(obstacle);
            std::vector<RVO::Vector2> obstacle1;
            obstacle1.push_back(RVO::Vector2(711,666));
            obstacle1.push_back(RVO::Vector2(711,647));
            obstacle1.push_back(RVO::Vector2(730,647));
            obstacle1.push_back(RVO::Vector2(730,666));
            sim->addObstacle(obstacle1);
        }

        /* Process the obstacles so that they are accounted for in the simulation. */
        sim->processObstacles();

        /* Set preferred velocities of agents */
        setPreferredVelocities(sim);

        /* Compute the next step of the simulation */
        std::cout << "Computing next step of RVO simulation" << std::endl;
        sim->doStep();

        geometry_msgs::PoseArray people_RVO;
        people_RVO.header.stamp = ros::Time(0);
        people_RVO.header.frame_id = "map";

        for (int i_col=0; i_col<detected_people.cols(); i_col++)
        {
            /* Agent 0 is the Ridgeback so starting with an offset of 1 */
            RVO::Vector2 agent = sim->getAgentPosition(1 + i_col);
            RVO::Vector2 agent_vel = sim->getAgentVelocity(1 + i_col);

            geometry_msgs::Pose person_RVO;

            /* Person's position */
            //person_RVO.position.x = agent.x();
            //person_RVO.position.y = agent.y();
            person_RVO.position.x = (agent.x() - start_cell_x + 0.25 * agent_vel.x()) * size_cell;
            person_RVO.position.y = (agent.y() - start_cell_y + 0.25 * agent_vel.y()) * size_cell;
            person_RVO.position.z = i_col;

            std::cout << "Agent " << 1 + i_col << " at position (" << agent.x() << "," << agent.y() << ")" << std::endl;
            std::cout << "In real world at position (" << person_RVO.position.x << "," << person_RVO.position.y << ")" << std::endl;
            std::cout << "With velocity (" << agent_vel.x() << "," << agent_vel.y() << ")" << std::endl;

            /* Person's orientation */
            // TODO: Change quaternion to align people with the direction they are going
            person_RVO.orientation.x = 0.0;
            person_RVO.orientation.y = 0.0;
            person_RVO.orientation.z = 0.0;
            person_RVO.orientation.w = 1.0;

            /* Add person to the array of poses */
            people_RVO.poses.push_back(person_RVO);

            // TODO: Update detected_people with new position and speed
            // Speed to 0???
            detected_people.col(i_col) << person_RVO.position.x, person_RVO.position.y, 0, 0, 0, detected_people(5,i_col), detected_people(6,i_col), 0, 0, 0;

        }

        /* Publish for Gazebo */
        pub_RVO.publish(people_RVO);

        /* Retrieving Ridgeback Velocity */
        RVO::Vector2 velocity_ridgeback;
        velocity_ridgeback = sim->getAgentVelocity(0);

        /* Assign velocity output */
        next_eps << velocity_ridgeback.x(), velocity_ridgeback.y(), 0.0;

        /* Delete simulation */
        delete sim;
    }


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
    // State next_eps = next_step_special_weighted( state_robot, state_attractor, storage, size_cell);
    ROS_INFO("VelCmd in map frame: %f %f %f", next_eps(0,0), next_eps(1,0), next_eps(2,0));

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

	    ROS_INFO("VelCmd in base_link frame: %f %f %f", vec3_stamped_transformed.x(), vec3_stamped_transformed.y(), next_eps(2,0));

	    // Creating velocity message
	    geometry_msgs::Twist output;
	    output.linear.x = vec3_stamped_transformed.x();
	    output.linear.y = vec3_stamped_transformed.y();
	    output.linear.z =  0.0;
	    output.angular.x = 0.0;
	    output.angular.y = 0.0;
	    output.angular.z = next_eps(2,0);

	    //ROS_INFO("VelCmd in Ridgeback frame: %f %f %f", next_eps(0,0), next_eps(1,0), next_eps(2,0));
	    //if ((static_cast<int>(key_command)!=5)&&(static_cast<int>(key_command)!=-1)) // if we press 5, the robot should not move
	   // {
            //pub_.publish(output);
	    //}

    }
    else
    {
            ROS_ERROR("Cannot transform velocity vector from map to base_link");
    }

    //////////////////////////
    // END OF MAIN CALLBACK //
    //////////////////////////
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

        // [x_c, y_c, v_x, v_y, stamp, x_goal, y_goal, 0, 0, 0]
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

void setPreferredVelocities(RVO::RVOSimulator *sim)
{
	/*
	 * Set the preferred velocity to be a vector of unit magnitude (speed) in the
	 * direction of the goal.
	 */

	for (size_t i = 0; i < sim->getNumAgents(); ++i) {
        //unsigned int k = static_cast<unsigned int>(i);

		RVO::Vector2 goalVector = goals[i] - sim->getAgentPosition(i);

		if (RVO::absSq(goalVector) > 1.0f) {
			goalVector = RVO::normalize(goalVector);
		}

		sim->setAgentPrefVelocity(i, goalVector);

		/*
		 * Perturb a little to avoid deadlocks due to perfect symmetry.
		 */
		float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
		float dist = std::rand() * 0.0001f / RAND_MAX;

		sim->setAgentPrefVelocity(i, sim->getAgentPrefVelocity(i) +
		                          dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
	}
}

void init_detected_people()
{
    // [x_c, y_c, v_x, v_y, stamp, x_goal, y_goal, 0, 0, 0]

    Eigen::MatrixXf temp_storage(10,2);

    temp_storage.col(0) << 5.0, 0.0, 0.0, 0.0, 0.0, 6.0, -11.5, 0, 0, 0.0;
    temp_storage.col(1) << 8.0, 0.0, 0.0, 0.0, 0.0, 9.0, -11.5, 0, 0, 0.0;

    detected_people = temp_storage;

    // TODO : Init people here

    // TODO : In gazebo plugin set position of people



}
private:
  ros::NodeHandle n_;
  ros::Publisher pub_;  // To publish the velocity command
  ros::Publisher pub_attractor;  // To publish the position of the attractor (for display)
  ros::Subscriber sub_; // To listen to the trigger topic
  ros::Subscriber sub_people; // To listen to the pose_people topic
  ros::Subscriber sub_key;    // To listen to the cmd_key topic
  tf::TransformListener listener_; // To listen to transforms
  tf::StampedTransform transform_; // Transform from map to base_link
  ros::ServiceClient client_map_;  // To call dynamic_map service of gmapping
  Eigen::MatrixXf detected_people;
  float key_command; // between 1 and 9 for direction
  int num_attractor; // numero of the attractor that is currently being followed


  /* Store the goals of the agents. */
  std::vector<RVO::Vector2> goals;

  ros::Publisher pub_RVO; // To publish people's position for Gazebo

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "ROV2_node");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  // Spin node
  ros::spin();

  return 0;
}


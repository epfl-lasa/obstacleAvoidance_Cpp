#include "ros/ros.h"
#include <tf/tf.h>
#include "tf/transform_listener.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Scalar.h"
#include "tf/LinearMath/Vector3.h"

// Info about OccupancyGrid and Float32MultiArray messages
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Float32MultiArray.h"

// Standard C library
#include <cstdlib>

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

        // Topic you want to subscribe (map topic to avoid service call)
        sub_map = n_.subscribe("/map", 3, &SubscribeAndPublish::callback_for_map, this);

        // Time start of node
        time_start = ros::WallTime::now().toSec();

        // State vector of the robot
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

    }

    ~SubscribeAndPublish()
    {
        // Destructor
    }

    void callback_for_map(const nav_msgs::OccupancyGrid& input) // Callback triggered by /map topic
    {
        float timestamp_timing = ros::Time::now().toSec();

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

        // Get rotation information between "map" and "base_link"
        tfScalar yaw, pitch, roll;
        tf::Matrix3x3 mat(transform_.getRotation());
        mat.getRPY(roll, pitch, yaw); // Assign values to roll pitch yaw variables

        // Create robot state vector and fill it
        state_robot << (transform_.getOrigin().getX() - x_pose)/ size_cell,
                    (transform_.getOrigin().getY() - y_pose)/ size_cell,
                    yaw;


        // ROS_INFO("Robot     | %f %f %f", state_robot(0,0), state_robot(1,0), state_robot(2,0));


        ///////////////////////////////
        // PROCESSING OCCUPANCY GRID //
        ///////////////////////////////

        //std::cout << " PROCESSING OCCUPANCY GRID " << std::endl;

        // Expand obstacles to get a security margin
        eig_expanded = expand_occupancy_grid( eig_test, n_expansion, state_robot, limit_in_cells, size_cell);

        auto t_process_grid = std::chrono::high_resolution_clock::now();

        /////////////////////////
        // DETECTING OBSTACLES //
        /////////////////////////

        //std::cout << " DETECTING OBSTACLES " << std::endl;

        // Detect expanded obstacles
        storage = detect_borders( eig_expanded, state_robot );

        auto t_detect_obs = std::chrono::high_resolution_clock::now();

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

        /*for (int i=0; i<storage.size(); i++)
        {
            std::cout << storage[i] << std::endl;
        }*/

        // Publish message
        pub_boundary.publish(array_msg);

    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_boundary;  // To publish the velocity command
    ros::Subscriber sub_map;    // To listen to the map topic
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


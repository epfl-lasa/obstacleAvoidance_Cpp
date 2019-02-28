#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
//#include "obstacle_avoidance/GetObstacles.h"
#include <cstdlib>

// Eigen library and the header that contains my functions
#include <eigen3/Eigen/Core>
#include "ObstacleAvoidance.h"

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<std_msgs::Float32>("obstacles_topic", 10);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/map", 1, &SubscribeAndPublish::callback, this);

    // Client of dynamic_map service of gmapping
    ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("dynamic_map");
  }

  void callback(const nav_msgs::OccupancyGrid& input)
  {
    // Get the transform between /map and /base_link
    try{
      listener_.lookupTransform("/base_link", "/map", ros::Time(0), transform_);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
    }

    // Call gmapping service
    nav_msgs::GetMap srv_map;
    if (client.call(srv_map))
    {
       ROS_INFO("Dynamic map received"); //map is in srv.response.map
    }
    else
    {
       ROS_ERROR("Failed to call dynamic map service");
    }

    // Get obstacles with map
    // TODO...

    // Get velocity command with obstacles and position of the robot (transform)
    // TODO...
    std_msgs::Float32 output;
    output.data = test_ros();
    ROS_INFO("Ouput received");
    pub_.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;  // To publish the velocity command
  ros::Subscriber sub_; // To listen to a topic (to be defined)
  tf::TransformListener listener_; // To listen to transforms
  tf::StampedTransform transform_; // Transform from map to base_link
  ros::ServiceClient client_map_; // To call dynamic_map service of gmapping

};//End of class SubscribeAndPublish


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

// Test function to see if one can pass data to a standard C++ function
void test_call(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("I also heard: [%f]", msg->ranges[0]);
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
   ROS_INFO("I heard: [%f]", msg->ranges[0]); // msg->data.c_str()
   test_call(msg);
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  ROS_INFO("I received the map.");
  // do something with the map
}

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "listener_data");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;


  // Subscriber to /front/scan topic to retrieve LaserScan data (front side)
  //ros::Subscriber sub_scan = n.subscribe("/front/scan", 100, scanCallback);
  
  // Subscriber to map topic to retrieve occupancy map
  //ros::Subscriber sub_map = n.subscribe("/map", 100, mapCallback);
  //ros::Subscriber sub_map = n.subscribe<nav_msgs::OccupancyGrid> ("/map", 10, 
                                   //boost::bind(processMapCallback, _1, argc, argv) );

  // Client handling to send the map to the GetObstacle server and retrieve the obstacles
  //ros::ServiceClient client = n.serviceClient<obstacle_avoidance::GetObstacles>("get_obstacles");
  //obstacle_avoidance::GetObstacles srv;


  ros::spin();

  return 0;
}

// Tutorial ROS : Writing a tf listener (C++)
// Class found in ros answers : "Publishing to a topic via subscriber callback function"
// How to initialize a UInt8MultiArray message

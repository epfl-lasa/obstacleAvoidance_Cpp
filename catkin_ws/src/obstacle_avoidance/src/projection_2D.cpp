#include "ros/ros.h"
#include <tf/tf.h>
#include "tf/transform_listener.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Scalar.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/PointCloud2.h"

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
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/velodyne_points", 1, &SubscribeAndPublish::callback, this);

    // Transform listener
    tf::TransformListener transform_(ros::Duration(10.0));
  }

  void callback(const sensor_msgs::PointCloud2& input)
  {
    
    // Get the transform between /map and /base_link
    try {
      listener_.lookupTransform("/base_link", "/map", ros::Time(0), transform_);
      ROS_INFO("Transform is ready");
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
    }

    // State of the robot
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(transform_.getRotation());
    mat.getRPY(roll, pitch, yaw);

    float X = (-1)*transform_.getOrigin().getX();
    float Y = (-1)*transform_.getOrigin().getY();
    float PHI = (-1)*yaw;

    ROS_INFO("Robot %f %f %f", X, Y, PHI);

    // Remove ground plane
    ROS_INFO("Test: %i", input.data[0]);


    //pub_.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;  // To publish the velocity command
  ros::Subscriber sub_; // To listen to a topic (to be defined)
  tf::TransformListener listener_; // To listen to transforms
  tf::StampedTransform transform_; // Transform from map to base_link

};//End of class SubscribeAndPublish


int maino(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "projection_2D");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}

// Tutorial ROS : Writing a tf listener (C++)
// Class found in ros answers : "Publishing to a topic via subscriber callback function"
// How to initialize a UInt8MultiArray message

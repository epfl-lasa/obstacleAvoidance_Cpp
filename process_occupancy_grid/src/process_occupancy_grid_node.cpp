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

//#include "obstacle_avoidance/GetObstacles.h"
#include <cstdlib>

// Eigen library and the header that contains my functions
#include <eigen3/Eigen/Core>
#include "ObstacleReconstruction.h"
#include "ObstacleAvoidance.h"

typedef Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXi8; // Dynamic Eigen matrix with type int8_t since OccupancyGrid contains int8_t values
typedef Eigen::Matrix<int8_t, 1, Eigen::Dynamic> MatrixXi8_layer;
typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> myMap;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/test", 1, &SubscribeAndPublish::callback, this);

    // Client of dynamic_map service of gmapping
    client_map_ = n_.serviceClient<nav_msgs::GetMap>("dynamic_map");

    // Transform listener
    tf::TransformListener transform_(ros::Duration(10.0));

  }

  void callback(const geometry_msgs::Twist& input)//const nav_msgs::OccupancyGrid& input)
  {

    // Call gmapping service
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
    // occupancy grid is now stored in the vector (1 dimension) with type int8_t

    // Convert std container to type int
    std::vector<int> int_vector(test_vector.begin(), test_vector.end());

    // Get std container into an Eigen matrix with the right width and height
    Eigen::Map<Eigen::MatrixXi> eig_test( int_vector.data(), srv_map.response.map.info.height, srv_map.response.map.info.width); 
    //std::cout << "Size after cast: " << eig_test.rows() << " " << eig_test.cols() << std::endl;
    

    float size_cell = 0.3;

    // Once the occupancy grid has been reshaped, c++ algorithms can be used
    
    // Expand obstacles to get a security margin
    eig_test = expand_occupancy_grid( eig_test, static_cast<int>(std::floor(1/size_cell)));
    
    std::cout << eig_test.block(340,310,41,41) << std::endl;

    // Detect expanded obstacles
    std::vector<Border> storage;
    storage = detect_borders( eig_test );
    /*for (int iter=0; iter < storage.size(); iter++)
    {
        std::cout << "Obstacle " << iter << ":"<< std::endl;
        std::cout << storage[iter] << std::endl;
    }*/
    std::cout << storage.size() << " obstacles have been detected." << std::endl;

    

    // NO NEED SINCE THE MAP IS USED AS THE REFERENCE FRAME
    // Retrieving Resolution and Pose of the map in the world frame
    float resolution = srv_map.response.map.info.resolution;
    float x_pose = srv_map.response.map.info.origin.position.x;
    float y_pose = srv_map.response.map.info.origin.position.y;
    ROS_INFO("Map origin: %f %f", x_pose, y_pose);
    /*tfScalar roll, pitch, yaw;
    tf::Matrix3x3 mat(srv_map.response.map.info.origin.orientation);
    mat.getRPY(roll, pitch, yaw);
    float phi_pose = yaw;*/

    // Get the transform between /map and /base_link (to get the position of the robot in the map)
    try{
      //listener_.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(10.0) );
      listener_.lookupTransform("map", "base_link", ros::Time(0), transform_);
      ROS_INFO("Transform is ready");
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
    }
    std::cout << "Translation X: " << transform_.getOrigin().getX() << std::endl;
    std::cout << "Translation Y: " << transform_.getOrigin().getY() << std::endl;
    // State of the robot
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(transform_.getRotation());
    mat.getRPY(roll, pitch, yaw);

    State state_robot; state_robot << (transform_.getOrigin().getX() - x_pose)/ size_cell, 
                                      (transform_.getOrigin().getY() - y_pose)/ size_cell,
                                      yaw;
    ROS_INFO("Robot %f %f %f", state_robot(0,0), state_robot(1,0), state_robot(2,0));

    // Map display centered on the robot
    /*std::cout << eig_test.block(static_cast<int>(std::floor(x_pose - transform_.getOrigin().getX()/size_cell)-20), static_cast<int>(std::floor(y_pose - transform_.getOrigin().getY()/size_cell)-20),41,41) << std::endl;*/

    // Set state of the attractor
    State state_attractor; state_attractor << 380,331,0;

    // Compute velocity command based on the detected obstacles
    State next_eps = next_step_several_obstacles_border( state_robot, state_attractor, storage);
    
    // Creating velocity message
    float scaling = 4;
    geometry_msgs::Twist output;
    output.linear.x = scaling * next_eps(0,0);
    output.linear.y = scaling * next_eps(1,0);
    output.linear.z = 0.0;
    output.angular.x = 0.0;
    output.angular.y = 0.0;
    output.angular.z = scaling * next_eps(2,0);
    
    ROS_INFO("VelCmd : %f %f %f", next_eps(0,0), next_eps(1,0), next_eps(2,0));
    pub_.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;  // To publish the velocity command
  ros::Subscriber sub_; // To listen to a topic (to be defined)
  tf::TransformListener listener_; // To listen to transforms
  tf::StampedTransform transform_; // Transform from map to base_link
  ros::ServiceClient client_map_;  // To call dynamic_map service of gmapping
  MatrixXi8 eig_map_;
  bool flag_map;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "process_occupancy_grid_node");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}

// Tutorial ROS : Writing a tf listener (C++)
// Class found in ros answers : "Publishing to a topic via subscriber callback function"
// How to initialize a UInt8MultiArray message

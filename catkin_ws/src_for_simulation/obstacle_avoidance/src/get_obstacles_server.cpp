#include "ros/ros.h"
#include "obstacle_avoidance/GetObstacles.h"
#include <home/leziart/Documents/eigen-v337/Eigen>
#include "home/leziart/Documents/Projet_28_02/include/ObstacleAvoidance.h"

std::vector<float> detectObstacles(obstacle_avoidance::GetObstacles::Request  &req,
         			   obstacle_avoidance::GetObstacles::Response &res)
{
  ROS_INFO("Obstacles requested");
  // res.obstacles = ?? 
  // with input req.map
  
  //ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  //ROS_INFO("sending back response: [%ld]", (long int)res.sum);

  int integer = test_ros();
  if (integer==42)
  {
     ROS_INFO("Test is successful.");
  }
  std::vector<float> placeholder;
  placeholder.push_back(43.0);
  return placeholder;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_obstacles_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("get_obstacles", detectObstacles);
  ROS_INFO("Ready to detect obstacles.");
  ros::spin();

  return 0;
}

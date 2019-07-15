#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"

/**
 * The goal of this node is to listen to the /cmd_vel which contains unstamped geometry_msgs::Twist messages
 * to emit them on the /cmd_vel_stamped with a timestamp as geometry_msgs::TwistStamped messages.
 * This timestamp is needed if you want to visualize the velocity command sent to the robot as vectors and Rviz
 * by adding a jsk_rviz_plugin/TwistStamped display in the left column of Rviz.
 */

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<geometry_msgs::TwistStamped>("/cmd_vel_stamped",1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/cmd_vel", 1, &SubscribeAndPublish::callback, this);
  }

void callback(const geometry_msgs::Twist& input)//const nav_msgs::OccupancyGrid& input)
{
    geometry_msgs::TwistStamped twist_stamped;

    twist_stamped.header.stamp = ros::Time::now();
    twist_stamped.header.frame_id = "base_link";
    twist_stamped.twist = input;

    pub_.publish(twist_stamped);
}

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;  // To publish the velocity command
  ros::Subscriber sub_; // To listen to a topic (to be defined)
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_cmd_vel");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cameras_tf_broadcaster");

  ros::NodeHandle node;

  // Transforms to go from the chassis frame to the frame of one of the cameras

  // TRANSFORM OF CAMERA 1 //

  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(-0.5, 0.0, 0.7) ); // X Y Z
  tf::Quaternion q;
  q.setRPY(0, 0, 0); // Roll Pitch Yaw
  transform.setRotation(q);

  // TRANSFORM OF CAMERA 2 //

  // ...

  ros::Rate loop_rate(30);
  while(ros::ok())
  {
       br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera_link"));
       loop_rate.sleep();               
  }

  return 0;
};

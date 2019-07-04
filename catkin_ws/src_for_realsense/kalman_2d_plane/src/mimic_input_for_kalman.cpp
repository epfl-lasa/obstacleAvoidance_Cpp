#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <random>

/* Structure of a geometry_msgs::PoseArray message
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose[] poses
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
*/

/*class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        // Publishing people's position inside and outside the FOV of Realsense cameras in velodyne frame
        pub_velodyne_ = n_.advertise<geometry_msgs::PoseArray>("/pose_people_velodyne", 2);

        seq_num = 1;

        ROS_INFO("mimic_input node has been initialized.");
    }

    void callback_camera_1(const geometry_msgs::PoseArray& input) // callback of /pose_people_camera_1
    {
        /////////////////////////////////////
        //// TRANSFORM TO VELODYNE FRAME ////
        /////////////////////////////////////

          // TRANSFORM OF CAMERA 1 //

  tf::TransformBroadcaster br;
  tf::Transform transform1;
  transform1.setOrigin( tf::Vector3(-0.5, 0.0, 0.7) ); // X Y Z
  tf::Quaternion q;
  q.setRPY(0, 0, 0); // Roll Pitch Yaw
  transform1.setRotation(q);
  //br.sendTransform(tf::StampedTransform(transform1, ros::Time(0), "base_link", "camera_link"));

        // Check if the pose can be transformed from the frame of the camera to the frame of the Velodyne
        bool can_transform_to_velodyne = false;
        try
        {
            listener_.lookupTransform("velodyne_link", "camera_link", ros::Time::now(), transform_);
            can_transform_to_velodyne = true;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
        }

        if (can_transform_to_velodyne)
        {   
            pose_people_in_velodyne.poses.clear(); // Remove previous poses from this camera

            for (int i=0; i<(input.poses).size(); i++) // For each person detected by this camera this camera
            {
                geometry_msgs::PoseStamped pose_person; 
                pose_person.header.seq = seq_num;
 		pose_person.header.frame_id = "velodyne_link";
                pose_person.header.stamp = ros::Time::now()
                pose_person.pose = input.poses[i];

                geometry_msgs::PoseStamped pose_person_in_velodyne; // PoseStamped that will received the transformed pose_person
                pose_person_in_velodyne.header = input.header;
                listener_.transformPose("velodyne_link", pose_person, pose_person_in_velodyne); // Transform from the frame of the camera to the one of the Velodyne

                pose_people_in_velodyne.poses.push_back(pose_person_in_velodyne.pose); // Append to array of poses
            }

            pose_people_in_velodyne.header = input.header;
            pub_velodyne_.publish(pose_people_in_velodyne); // Publish poses in velodyne frame
        }
    }


private:
    ros::NodeHandle n_;              // Node handle
    ros::Publisher pub_velodyne_;    // To publish the transformed people's position into velodyne frame
    tf::TransformListener listener_; // To listen to transforms
    tf::StampedTransform transform_; // Transform object

    

    geometry_msgs::PoseArray pose_people_in_velodyne;

};//End of class SubscribeAndPublish*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mimic_input_for_kalman");

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0,1);

    ros::NodeHandle n_;              // Node handle
    ros::Publisher pub_velodyne_;    // To publish the transformed people's position into velodyne frame
    pub_velodyne_ = n_.advertise<geometry_msgs::PoseArray>("/pose_people_velodyne", 2);
    int seq_num = 0;
    geometry_msgs::PoseArray pose_people_in_velodyne;
    double t_start = ros::Time::now().toSec();
    ROS_INFO("mimic_input node has been initialized.");

    ros::Rate loop_rate(4);
    while(ros::ok())
    {
         double t_current = ros::Time::now().toSec();
         float t_max = 6.0f;
         float t = std::fmod(static_cast<float>(t_current-t_start), t_max);
         ROS_INFO("Time: %f", t);
         pose_people_in_velodyne.poses.clear(); // Remove previous poses from this camera

         seq_num++;
         pose_people_in_velodyne.header.seq = seq_num;
 	 pose_people_in_velodyne.header.frame_id = "velodyne_link";
         pose_people_in_velodyne.header.stamp = ros::Time::now();

         geometry_msgs::PoseStamped pose_person;

	 // PERSON 1
         if (dis(gen)==1) {
         if (t<(t_max*0.5))
         { 
         pose_person.pose.position.x = t;
         pose_person.pose.position.y = std::min(t,-t+static_cast<float>(t_max*0.5));
         } 
         else
         { 
         pose_person.pose.position.x = t_max - t;
         pose_person.pose.position.y = std::max(t-t_max,-t+static_cast<float>(t_max*0.5));
         } 
         pose_person.pose.position.z = 0.0f;
         pose_person.pose.orientation.x = 0.0f;
         pose_person.pose.orientation.y = 0.0f;
         pose_person.pose.orientation.z = 0.0f;
         pose_person.pose.orientation.w = 1.0f;
         pose_people_in_velodyne.poses.push_back(pose_person.pose); }

	 // PERSON 2
         if (dis(gen)==1) {
         if (t<(t_max*0.5))
         { 
         pose_person.pose.position.x = t;
         pose_person.pose.position.y = std::min(t,-t+static_cast<float>(t_max*0.5)) - 0.5;
         } 
         else
         { 
         pose_person.pose.position.x = t_max - t;
         pose_person.pose.position.y = std::max(t-t_max,-t+static_cast<float>(t_max*0.5)) - 0.5;
         } 
         pose_person.pose.position.z = 0.0f;
         pose_people_in_velodyne.poses.push_back(pose_person.pose); }
      
         // Publish PoseArray
         pub_velodyne_.publish(pose_people_in_velodyne);

         // Sleep to get the right loop frequency
         loop_rate.sleep();               
    }

    return 0;
}



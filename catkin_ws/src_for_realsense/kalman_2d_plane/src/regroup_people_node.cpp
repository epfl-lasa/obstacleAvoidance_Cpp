#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

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

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        // Publishing people's position inside and outside the FOV of Realsense cameras in map frame
        pub_map_ = n_.advertise<geometry_msgs::PoseArray>("/pose_people_map", 2);

        // Publishing people's position inside and outside the FOV of Realsense cameras in velodyne frame
        pub_velodyne_ = n_.advertise<geometry_msgs::PoseArray>("/pose_people_velodyne", 2);

        // Subscribing to people's position detected by Realsense cameras
        sub_ = n_.subscribe("/pose_people", 1, &SubscribeAndPublish::callback_camera_1, this);

        // Initialisation of class variables
        //pose_person_in_velodyne = geometry_msgs::PoseArray();
        //pose_person_in_map = geometry_msgs::PoseArray();
        /*pose_person_in_velodyne.header.seq = 1;
        pose_person_in_velodyne.header.stamp = ros::Time(0);
        pose_person_in_velodyne.header.frame_id = "velodyne;
        pose_person_in_velodyne.poses = std::vector<geometry_msgs::Pose>;*/

        ROS_INFO("regroup_people node has been initialized.");
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
                geometry_msgs::PoseStamped pose_person; // I have to use a PoseStamped because tf cannot work with PoseArray
                pose_person.header = input.header;
                pose_person.pose = input.poses[i];

                geometry_msgs::PoseStamped pose_person_in_velodyne; // PoseStamped that will received the transformed pose_person
                pose_person_in_velodyne.header = input.header;
                listener_.transformPose("velodyne_link", pose_person, pose_person_in_velodyne); // Transform from the frame of the camera to the one of the Velodyne

                pose_people_in_velodyne.poses.push_back(pose_person_in_velodyne.pose); // Append to array of poses
            }

            pose_people_in_velodyne.header = input.header;
            pub_velodyne_.publish(pose_people_in_velodyne); // Publish poses in velodyne frame
        }

        ////////////////////////////////
        //// TRANSFORM TO MAP FRAME ////
        ////////////////////////////////

        // Check if the pose can be transformed from the frame of the camera to the frame of the map
        /*bool can_transform_to_map = false;
        try
        {
            listener_.lookupTransform("map", "camera_color_optical_frame", ros::Time(0), transform_);
            can_transform_to_map = true;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
        }

        if (can_transform_to_map)
        {
            pose_people_in_map.poses.clear(); // Remove previous poses from this camera

            for (int i=0; i<(input.poses).size(); i++) // For each person detected by this camera this camera
            {
                geometry_msgs::PoseStamped pose_person; // I have to use a PoseStamped because tf cannot work with PoseArray
                pose_person.header = input.header;
                pose_person.pose = input.poses[i];

                geometry_msgs::PoseStamped pose_person_in_map; // PoseStamped that will received the transformed pose_person
                pose_person_in_map.header = input.header;
                // listener_.tranformPose("map", pose_person, pose_person_in_map); // Transform from the frame of the camera to the one of the map

                pose_people_in_map.poses.push_back(pose_person_in_map.pose); // Append to array of poses
            }

            pose_people_in_map.header = input.header;
            pub_map_.publish(pose_people_in_map); // Publish poses in map frame
        }*/
    }

    /*void callback_camera_i(const geometry_msgs::PoseArray& input) // callback of /pose_people_camera_i
    {
        // Same as callback_camera_1
        // Add a callback for each camera with a subscriber
    }*/

private:
    ros::NodeHandle n_;              // Node handle
    ros::Publisher pub_map_;         // To publish the transformed people's position into map frame
    ros::Publisher pub_velodyne_;    // To publish the transformed people's position into velodyne frame
    ros::Subscriber sub_;            // To listen to people's position detected by Realsense cameras
    tf::TransformListener listener_; // To listen to transforms
    tf::StampedTransform transform_; // Transform object

    geometry_msgs::PoseArray pose_people_in_velodyne;
    geometry_msgs::PoseArray pose_people_in_map;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
    ros::init(argc, argv, "regroup_people_node");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    ros::spin();

    return 0;
}



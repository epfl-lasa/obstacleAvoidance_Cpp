#include "ros/ros.h"
#include <tf/tf.h>
#include "tf/transform_listener.h"

#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3Stamped.h"

#include <cstdlib>
#include <vector>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_velodyne = n_.advertise<geometry_msgs::PoseArray>("/pose_people", 10);

    //Topic you want to publish
    pub_map = n_.advertise<geometry_msgs::PoseArray>("/pose_people_map", 10);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/gazebo/pos_moving_people", 1, &SubscribeAndPublish::callback, this);

    // Transform listener
    tf::TransformListener transform_(ros::Duration(10.0));


    // Placeholder person to fill the vector of people
    placeholder.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0); 
    placeholder.position.x = 0.0;
    placeholder.position.y = 0.0;
    placeholder.position.z = 0.0;

    ROS_INFO("vector_to_posearray node has been initialized");
    ROS_INFO("Converting /gazebo/pos_moving_people Vector3Stamped to /pose_people PoseArray");
  }

  void callback(const geometry_msgs::Vector3Stamped& input) // callback is triggered by /test topic
  {
    ///////////////////
    // MAIN CALLBACK //
    ///////////////////
    
    // Pose array
    geometry_msgs::PoseArray people_velodyne, people_map;
    people_velodyne.header.stamp = ros::Time::now();
    people_velodyne.header.frame_id = "velodyne";
    people_map.header = people_velodyne.header;
    people_map.header.frame_id = "map";

    // Pose object
    geometry_msgs::Pose pose_person;
    
    // Default orientation
    pose_person.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0); 

    // Get position data from received message
    pose_person.position.x = input.vector.x;
    pose_person.position.y = input.vector.y;
    pose_person.position.z = 0.0;
    
    // Updating size of table_people_ if needed
    if (static_cast<int>(std::round(input.vector.z)) > table_people_velodyne.size() )
    {
         int current_size = table_people_velodyne.size();
	 for (int i=0; i < (static_cast<int>(input.vector.z)-current_size); i++)
         {
             table_people_velodyne.push_back(placeholder);
             table_people_map.push_back(placeholder);
         }
    }
    

  
    // Get the transform between /map and /base_link (to get the position of the robot in the map)
    ros::Time t = ros::Time(0);
    try
    {
      listener_.lookupTransform("map", "velodyne", t, transform_);
      //ROS_INFO("Transform is ready");
    
    // Go from "map" to "base_link"
    if ( listener_.canTransform("velodyne", "map", t))
    {
            // Updating information of table_people_map
	    if (table_people_map.size() > 0)
	    {
	       table_people_map[static_cast<int>(input.vector.z)-1] = pose_person;
	    } 

            // Need to create a PoseStamped to use transformPose
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.pose.position = pose_person.position;
	    pose_stamped.pose.orientation = pose_person.orientation;
            pose_stamped.header.stamp = t;
            pose_stamped.header.frame_id = "map";
	    geometry_msgs::PoseStamped pose_stamped_transformed = pose_stamped;	

 	    // Transform from "map" to "velodyne"
            listener_.transformPose("velodyne", pose_stamped, pose_stamped_transformed);
       
            // Retrieve transformed data
            pose_person.position    = pose_stamped_transformed.pose.position;
            pose_person.orientation = pose_stamped_transformed.pose.orientation;

	    // Updating information of table_people_velodyne
	    if (table_people_velodyne.size() > 0)
	    {
	       table_people_velodyne[static_cast<int>(input.vector.z)-1] = pose_person; // now it is in /velodyne frame
	    } 
	    
	    // Stacking people in the array
	    for (int k=0; k<table_people_velodyne.size(); k++)
	    {
	    	people_velodyne.poses.push_back(table_people_velodyne[k]);
                people_map.poses.push_back(table_people_map[k]);
	    }

	    // Publishing pose array
	    pub_velodyne.publish(people_velodyne);
            pub_map.publish(people_map);
    }
    else
    {
            ROS_ERROR("Cannot tramsform velocity vector from map to velodyne");
    }

    }
    catch (tf::TransformException &ex) 
    {
      ROS_ERROR("%s",ex.what());
    }

    //////////////////////////
    // END OF MAIN CALLBACK //
    //////////////////////////
  }

private:
  ros::NodeHandle n_;   // ROS node handle
  ros::Publisher pub_velodyne;  // To publish the pose array in /velodyne frame
  ros::Publisher pub_map;       // To publish the pose array in /map frame
  ros::Subscriber sub_; // To listen to the pose messages
  std::vector<geometry_msgs::Pose> table_people_velodyne; // To store information about poses in /velodyne frame
  std::vector<geometry_msgs::Pose> table_people_map; // To store information about poses in /map frame
  geometry_msgs::Pose placeholder; // Placeholder pose
  tf::TransformListener listener_; // To listen to transforms
  tf::StampedTransform transform_; // Transform from map to base_link

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "vector_to_posearray_node");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  // Spin node
  ros::spin();

  return 0;
}


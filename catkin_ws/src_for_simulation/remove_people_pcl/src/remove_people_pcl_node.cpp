#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <cstdlib>
#include <cmath>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// For synchronization of subscribed topics
//#include <message_filters/subscriber.h>
//#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>


class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        //Topic you want to publish
        pub_ = n_.advertise<sensor_msgs::PointCloud2>("/cloud_without_people", 1);

        //Topic you want to subscribe
        sub_poses_ = n_.subscribe("/pose_people_velodyne_filtered", 2, &SubscribeAndPublish::callback_poses, this);
   
        //Topic you want to subscribe
        sub_pcl_ = n_.subscribe("/cloud_for_static_no_arm", 2, &SubscribeAndPublish::callback_pcl, this);
        
        ROS_INFO("remove_people_plc_node node has been initialized.");
    }

    void callback_pcl(const sensor_msgs::PointCloud2& input)
    {

    // Point cloud that will be published after filtering
    sensor_msgs::PointCloud2 output = input;

    for (int i=0; i<people.poses.size(); i++)
    {
        // TODO: implement for loop that read content of people array and test for each one of them
        /*geometry_msgs::PoseStamped pose_person;
        pose_person.header = people.header;
        pose_person.pose = people.poses[i];
        geometry_msgs::PoseStamped pose_person_in_map;
        pose_person_in_map.header = people.header;*/
        //transform_.tranformPose("map", pose_person, pose_person_in_map)

        float personX = (people.poses[i]).position.x;
        float personY = (people.poses[i]).position.y;
        float personZ = (people.poses[i]).position.z;

        std::cout << "Remove around (X,Y,Z) = (" << personX << "," << personY << "," << personZ << ")" << std::endl;

        for (sensor_msgs::PointCloud2Iterator<float> it(output, "x"); it != it.end(); ++it)
        {
            float cloudPointX = it[0];
            float cloudPointY = it[1];
            float cloudPointZ = it[2];

            // Compute horizontal distance to the person
            float distance = std::sqrt(std::pow((cloudPointX - personX), 2) + std::pow((cloudPointY - personY), 2));

            // Remove all points in a cylinder around the person
            if(distance < 0.5)
            {
                it[0] = std::numeric_limits<float>::quiet_NaN();
                it[1] = std::numeric_limits<float>::quiet_NaN();
                it[2] = std::numeric_limits<float>::quiet_NaN();
            }

        }
    }
    // pc2 has been filtered, output can be published
    pub_.publish(output);
    }

    void callback_poses(const geometry_msgs::PoseArray& input)
    {
         people = input;
    }

private:
    ros::NodeHandle n_;   // Node handle
    ros::Publisher pub_;  // To publish filtered boxes
    ros::Subscriber sub_poses_; // To listen to raw boxes
    ros::Subscriber sub_pcl_; // To listen to raw boxes
    tf::TransformListener listener_;
    tf::Transform transform_; // Transform object

    geometry_msgs::PoseArray people;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "remove_people_pcl_node");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    ros::spin();

    return 0;
}


#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include <cstdlib>
#include <cmath>
#include <tf/transform_listener.h>

// For synchronization of subscribed topics
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;

ros::Publisher pub;

void callback(const geometry_msgs::PoseArray::ConstPtr& people, const sensor_msgs::PointCloud2::ConstPtr& pc2)
{

    

    // Point cloud that will be published after filtering
    sensor_msgs::PointCloud2 output = *pc2;

    for (int i=0; i<people->poses.size(); i++)
    {
        // TODO: implement for loop that read content of people array and test for each one of them
        geometry_msgs::PoseStamped pose_person;
        pose_person.header = people->header;
        pose_person.pose = people->poses[i];
        geometry_msgs::PoseStamped pose_person_in_map;
        pose_person_in_map.header = people->header;
        //listener.tranformPose("map", pose_person, pose_person_in_map)

        float personX = pose_person.pose.position.x;
        float personY = pose_person.pose.position.y;
        float personZ = pose_person.pose.position.z;

        std::cout << "Remove around (X,Y,Z) = (" << personX << "," << personY << "," << personZ << ")" << std::endl;

        for (sensor_msgs::PointCloud2Iterator<float> it(output, "x"); it != it.end(); ++it)
        {
            float cloudPointX = it[0];
            float cloudPointY = it[1];
            float cloudPointZ = it[2];

            // Compute horizontal distance to the person
            float distance = std::sqrt(std::pow((cloudPointX - personX), 2) + std::pow((cloudPointZ - personZ), 2));

            // Remove all points in a cylinder around the person
            if(distance < 1.0)
            {
                it[0] = std::numeric_limits<float>::quiet_NaN();
                it[1] = std::numeric_limits<float>::quiet_NaN();
                it[2] = std::numeric_limits<float>::quiet_NaN();
            }

        }
    }
    // pc2 has been filtered, output can be published
    pub.publish(output);
}

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "remove_people_pcl_node");

    // Node handle
    ros::NodeHandle n;

    // Subscriber to PoseArray topic
    message_filters::Subscriber<geometry_msgs::PoseArray> sub_1(n, "/pose_people_in_velodyne", 2);

    // Subscriber to PointCloud2 topic
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_2(n, "/velodyne_points", 2);
    // TODO: For easier test with camera/depth_register/points but in reality with velodyne points

    // Synchronization policy
    typedef sync_policies::ApproximateTime<geometry_msgs::PoseArray, sensor_msgs::PointCloud2> MySyncPolicy;

    // ApproximateTime takes a queue size as its constructor argument,
    // hence MySyncPolicy(10). Synchronizer for the two topics:
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_1, sub_2);

    // Link synchronizer with callback
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // Publisher of the filtered PointCloud2
    pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_without_people", 1);

    // Transform listener
    tf::TransformListener listener;

    ros::spin();

    return 0;
}




#include "ros/ros.h"

#include <rosbag/bag.h>

#include "sensor_msgs/PointCloud2.h"    // For /velodyne_points
#include "sensor_msgs/LaserScan.h"      // For /scan
#include "sensor_msgs/Image.h"          // For /camera/color/image_raw

#include <eigen3/Eigen/Core>


class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        ROS_INFO("Starting node");

        bool log_cloud, log_laserscan, log_image;
        float freq_cloud, freq_laserscan, freq_image;

        // Time start of node
        int time_start = static_cast<int>(std::round(ros::WallTime::now().toSec()));

        // Check if the user wants to record the pointcloud
        n_.param("/log_data_sensors_node/log_cloud", log_cloud, false);
        n_.param("/log_data_sensors_node/freq_cloud", freq_cloud, 10.0f);
        period_cloud = 1 / freq_cloud;

        // Subscriber to PointCloud2 topic
        if (log_cloud)
        {
            ROS_INFO("Logging point cloud at %f Hz", freq_cloud);
            sub_cloud = n_.subscribe("/velodyne_points", 1, &SubscribeAndPublish::callback_cloud, this);
            bag_cloud.open("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Logging/data_cloud_"+std::to_string(time_start)+".bag", rosbag::bagmode::Write);
        }

        // Check if the user wants to record the laserscan
        n_.param("/log_data_sensors_node/log_laserscan", log_laserscan, false);
        n_.param("/log_data_sensors_node/freq_laserscan", freq_laserscan, 10.0f);
        period_laserscan = 1 / freq_laserscan;

        // Subscriber to LaserScan topic
        if (log_laserscan)
        {
            ROS_INFO("Logging laserscan at %f Hz", freq_laserscan);
            sub_laserscan = n_.subscribe("/scan", 1, &SubscribeAndPublish::callback_laserscan, this);
            bag_laserscan.open("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Logging/data_laserscan_"+std::to_string(time_start)+".bag", rosbag::bagmode::Write);
        }

        // Check if the user wants to record the image
        n_.param("/log_data_sensors_node/log_image", log_image, false);
        n_.param("/log_data_sensors_node/freq_image", freq_image, 10.0f);
        period_image = 1 / freq_image;

        // Subscriber to LaserScan topic
        if (log_image)
        {
            ROS_INFO("Logging video stream at %f Hz", freq_image);
            sub_image = n_.subscribe("/camera/color/image_raw", 1, &SubscribeAndPublish::callback_image, this);
            bag_image.open("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Logging/data_image_"+std::to_string(time_start)+".bag", rosbag::bagmode::Write);
        }

        timer_cloud = 0; timer_laserscan = 0; timer_image = 0;

        ROS_INFO("Data files for sensors have been opened");
        ROS_INFO("Recording");
    }

    ~SubscribeAndPublish()
    {
        ROS_INFO("Terminating node");
        bag_cloud.close();
        bag_laserscan.close();
        bag_image.close();
        ROS_INFO("Data files for sensors have been opened");
    }

    void callback_cloud(const sensor_msgs::PointCloud2& input)
    {
        t_now = ros::Time::now();
        if ((t_now.toSec()-timer_cloud) > period_cloud)
        {
            bag_cloud.write("/velodyne_points", t_now, input);
            timer_cloud = t_now.toSec();
        }
    }

    void callback_laserscan(const sensor_msgs::LaserScan& input)
    {
        t_now = ros::Time::now();
        if ((t_now.toSec()-timer_laserscan) > period_laserscan)
        {
            bag_laserscan.write("/scan", t_now, input);
            timer_laserscan = t_now.toSec();
        }
    }

    void callback_image(const sensor_msgs::Image& input)
    {
        t_now = ros::Time::now();
        if ((t_now.toSec()-timer_image) > period_image)
        {
            bag_image.write("/camera/color/image_raw", t_now, input);
            timer_image = t_now.toSec();
        }
    }

private:
    ros::NodeHandle n_; // ROS node

    // Log a PointCloud2
    ros::Subscriber sub_cloud;
    rosbag::Bag bag_cloud;

    // Log a LaserScan
    ros::Subscriber sub_laserscan;
    rosbag::Bag bag_laserscan;

    // Log am Image stream
    ros::Subscriber sub_image;
    rosbag::Bag bag_image;

    // Timers
    float timer_cloud, timer_laserscan, timer_image;

    // Periods
    float period_cloud, period_laserscan, period_image;

    ros::Time t_now;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "log_data_sensors_node");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    ros::spin();

    return 0;
}


#include <ros/ros.h>
#include <string.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "sensor_msgs/LaserScan.h"
#include "pcl_ros/point_cloud.h"
#include <Eigen/Dense>
#include <dynamic_reconfigure/server.h>
#include <ira_laser_tools/laserscan_multi_mergerConfig.h>
#include "geometry_msgs/PoseArray.h"

using namespace std;
using namespace pcl;
using namespace laserscan_multi_merger;

class LaserscanMerger
{
public:
    LaserscanMerger();
    void callback_poses(const geometry_msgs::PoseArray& input); // Callback triggered when an array containing people's position in velodyne frame is received
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic);
    void pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud);
    void reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level);

private:
    ros::NodeHandle node_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;

    ros::Publisher point_cloud_publisher_;
    ros::Publisher laser_scan_publisher_;
    vector<ros::Subscriber> scan_subscribers;
    vector<bool> clouds_modified;

    vector<pcl::PCLPointCloud2> clouds;
    vector<string> input_topics;

    void laserscan_topic_parser();

    double angle_min;
    double angle_max;
    double angle_increment;
    double time_increment;
    double scan_time;
    double range_min;
    double range_max;

    string destination_frame;
    string cloud_destination_topic;
    string scan_destination_topic;
    string laserscan_topics;

    // Additional variables to remove people from the input scans
    ros::Subscriber posearray_subscriber; // Subscriber to listen to people's position in velodyne frame
    geometry_msgs::PoseArray people;      // Variable in which people's position in velodyne frame are stored
    tf::TransformListener listener_;      // To listen to transforms
    tf::StampedTransform transform_;      // Transform object
};

void LaserscanMerger::reconfigureCallback(laserscan_multi_mergerConfig &config, uint32_t level)
{
	this->angle_min = config.angle_min;
	this->angle_max = config.angle_max;
	this->angle_increment = config.angle_increment;
	this->time_increment = config.time_increment;
	this->scan_time = config.scan_time;
	this->range_min = config.range_min;
	this->range_max = config.range_max;
}

void LaserscanMerger::laserscan_topic_parser()
{
	// LaserScan topics to subscribe
	ros::master::V_TopicInfo topics;
	ros::master::getTopics(topics);

    istringstream iss(laserscan_topics);
	vector<string> tokens;
	copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter<vector<string> >(tokens));

	vector<string> tmp_input_topics;

	for(int i=0;i<tokens.size();++i)
	{
	        for(int j=0;j<topics.size();++j)
		{
			if( (tokens[i].compare(topics[j].name) == 0) && (topics[j].datatype.compare("sensor_msgs/LaserScan") == 0) )
			{
				tmp_input_topics.push_back(topics[j].name);
			}
		}
	}

	sort(tmp_input_topics.begin(),tmp_input_topics.end());
	std::vector<string>::iterator last = std::unique(tmp_input_topics.begin(), tmp_input_topics.end());
	tmp_input_topics.erase(last, tmp_input_topics.end());

	ROS_WARN("PASS %s", tmp_input_topics[0].c_str());
	// Do not re-subscribe if the topics are the same
	if( (tmp_input_topics.size() != input_topics.size()) || !equal(tmp_input_topics.begin(),tmp_input_topics.end(),input_topics.begin()))
	{

		// Unsubscribe from previous topics
		for(int i=0; i<scan_subscribers.size(); ++i)
			scan_subscribers[i].shutdown();

		input_topics = tmp_input_topics;
		if(input_topics.size() > 0)
		{
            scan_subscribers.resize(input_topics.size());
			clouds_modified.resize(input_topics.size());
			clouds.resize(input_topics.size());
            ROS_WARN("Subscribing to topics\t%ld", scan_subscribers.size());
			for(int i=0; i<input_topics.size(); ++i)
			{
                scan_subscribers[i] = node_.subscribe<sensor_msgs::LaserScan> (input_topics[i].c_str(), 1, boost::bind(&LaserscanMerger::scanCallback,this, _1, input_topics[i]));
				clouds_modified[i] = false;
				cout << input_topics[i] << " ";
			}
		}
		else
            ROS_INFO("Not subscribed to any topic.");
	}
}

LaserscanMerger::LaserscanMerger()
{
	ros::NodeHandle nh("~");

	nh.getParam("destination_frame", destination_frame);
	nh.getParam("cloud_destination_topic", cloud_destination_topic);
	nh.getParam("scan_destination_topic", scan_destination_topic);
    nh.getParam("laserscan_topics", laserscan_topics);

    nh.param("angle_min", angle_min, -2.36);
    nh.param("angle_max", angle_max, 2.36);
    nh.param("angle_increment", angle_increment, 0.0058);
    nh.param("scan_time", scan_time, 0.0333333);
    nh.param("range_min", range_min, 0.45);
    nh.param("range_max", range_max, 25.0);
    ROS_INFO("laserscan_topics is %s", laserscan_topics.c_str());
    ROS_INFO("cloud_destination_topic is %s", cloud_destination_topic.c_str());
    this->laserscan_topic_parser();

    posearray_subscriber = node_.subscribe("/pose_people_velodyne_filtered", 2, &LaserscanMerger::callback_poses, this); // Subscriber to listen to people's position in velodyne frame
	point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> (cloud_destination_topic.c_str(), 1, false);
	laser_scan_publisher_ = node_.advertise<sensor_msgs::LaserScan> (scan_destination_topic.c_str(), 1, false);

}

/**
 * Callback function that is triggered when the node receives an array containing detected people's position in velodyne_link frame (topic /pose_people_velodyne_filtered)
 * Store people's position in "people" variable after transforming them from velodyne_link frame to base_link frame
 */
void LaserscanMerger::callback_poses(const geometry_msgs::PoseArray& input)
{

        ros::Time now = ros::Time::now();
        bool can_transform_to_baselink = false;
        try
        {
            listener_.waitForTransform("base_link", "velodyne_link", now, ros::Duration(2.0));
            can_transform_to_baselink = true;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
        }

        if (can_transform_to_baselink)
        {
            people.poses.clear(); // Remove previous poses

            for (int i=0; i<(input.poses).size(); i++) // For each person detected
            {
                geometry_msgs::PoseStamped pose_person; // I have to use a PoseStamped because tf cannot work with PoseArray
                pose_person.header = input.header;
                pose_person.header.frame_id="velodyne_link";
                pose_person.header.stamp=now;
                pose_person.pose = input.poses[i];

                geometry_msgs::PoseStamped pose_person_in_baselink; // PoseStamped that will received the transformed pose_person
                pose_person_in_baselink.header = input.header;

                listener_.transformPose("base_link", pose_person, pose_person_in_baselink); // Transform from the frame of the velodyne to the one of "base_link"

                people.poses.push_back(pose_person_in_baselink.pose); // Append to array of poses
            }

            people.header = input.header;
            people.header.frame_id = "base_link";
        }
}

/**
 * Callback function that merges sensor_msgs::LaserScan messages into a single 3D point cloud of type pcl::PCLPointCloud2
 */
void LaserscanMerger::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan, std::string topic)
{

	sensor_msgs::PointCloud tmpCloud1,tmpCloud2;
	sensor_msgs::PointCloud2 tmpCloud3;

    // Verify that TF knows how to transform from the received scan to the destination scan frame
	tfListener_.waitForTransform(scan->header.frame_id.c_str(), destination_frame.c_str(), scan->header.stamp, ros::Duration(1));
    projector_.transformLaserScanToPointCloud(scan->header.frame_id, *scan, tmpCloud1, tfListener_, laser_geometry::channel_option::Distance);
	try
	{
		tfListener_.transformPointCloud(destination_frame.c_str(), tmpCloud1, tmpCloud2);
	}catch (tf::TransformException ex){ROS_ERROR("%s",ex.what());return;}

	for(int i=0; i<input_topics.size(); ++i)
	{
		if(topic.compare(input_topics[i]) == 0)
		{
			sensor_msgs::convertPointCloudToPointCloud2(tmpCloud2,tmpCloud3);
			pcl_conversions::toPCL(tmpCloud3, clouds[i]);
			clouds_modified[i] = true;
		}
	}

    // Count how many scans we have
	int totalClouds = 0;
	for(int i=0; i<clouds_modified.size(); ++i)
		if(clouds_modified[i])
			++totalClouds;

    // Go ahead only if all subscribed scans have arrived
	if(totalClouds == clouds_modified.size())
	{
		pcl::PCLPointCloud2 merged_cloud = clouds[0];
		clouds_modified[0] = false;

		for(int i=1; i<clouds_modified.size(); ++i)
		{
			pcl::concatenatePointCloud(merged_cloud, clouds[i], merged_cloud);
			clouds_modified[i] = false;
		}

		point_cloud_publisher_.publish(merged_cloud);

		Eigen::MatrixXf points;
		getPointCloudAsEigen(merged_cloud,points);

		pointcloud_to_laserscan(points, &merged_cloud);
	}
}

/**
 * Callback function that transforms the merged 3D point cloud of type pcl::PCLPointCloud2 into a single sensor_msgs::LaserScan message after discarding unwanted points
 */
void LaserscanMerger::pointcloud_to_laserscan(Eigen::MatrixXf points, pcl::PCLPointCloud2 *merged_cloud)
{
	sensor_msgs::LaserScanPtr output(new sensor_msgs::LaserScan());
	output->header = pcl_conversions::fromPCL(merged_cloud->header);
	output->header.frame_id = destination_frame.c_str();
	output->header.stamp = ros::Time::now();  //fixes #265
	output->angle_min = this->angle_min;
	output->angle_max = this->angle_max;
	output->angle_increment = this->angle_increment;
	output->time_increment = this->time_increment;
	output->scan_time = this->scan_time;
	output->range_min = this->range_min;
	output->range_max = this->range_max;

	uint32_t ranges_size = std::ceil((output->angle_max - output->angle_min) / output->angle_increment);
	output->ranges.assign(ranges_size, output->range_max + 1.0); // - 0.01);//

	for(int i=0; i<points.cols(); i++)
	{
		const float &x = points(0,i);
		const float &y = points(1,i);
		const float &z = points(2,i);

		// Discard points if they have a Not A Number value
		if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
		{
			ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", x, y, z);
			continue;
		}

		// Discard points if they are too close from the robot
		double range_sq = y*y+x*x;
		double range_min_sq_ = output->range_min * output->range_min;
		if (range_sq < range_min_sq_) {
			ROS_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
			continue;
		}

		// Discard points if they are too far from the robot
		double angle = atan2(y, x);
		if (angle < output->angle_min || angle > output->angle_max)
		{
			ROS_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output->angle_min, output->angle_max);
			continue;
		}

        // Discard points belonging to the UR5 arm if it is deployed (discard every points in a box centered on the Ridgeback)
        float const min_x = -0.5;
        float const min_y = -0.5;
        float const max_x =  0.5;
        float const max_y =  0.5;
        if ((x>min_x)&&(x<max_x)&&(y>min_y)&&(y<max_y))
        {
            continue;
        }

        // Discard points belonging to detected people (points in a cylinder around people's estimated positions)
        bool flag_in_cylinder = false;
        int k = 0;
        while ((k<people.poses.size()) && !(flag_in_cylinder))
        {
            float personX = (people.poses[k]).position.x;
            float personY = (people.poses[k]).position.y;
            float personZ = (people.poses[k]).position.z;

            float const radius_around_people = 0.8; // in [m]

            // Compute horizontal distance to the person
            float distance = std::sqrt(std::pow((x - personX), 2) + std::pow((y - personY), 2));

            // Remove all points in a cylinder around the person
            if(distance < radius_around_people)
            {
                flag_in_cylinder = true;
            }
            k++;
        }
		if (flag_in_cylinder) {continue;} // point discarded as it is in a cylinder

        // If point has not been discarded
		int index = (angle - output->angle_min) / output->angle_increment;

		if (output->ranges[index] * output->ranges[index] > range_sq)
			output->ranges[index] = sqrt(range_sq);
	}
	laser_scan_publisher_.publish(output);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_multi_merger");

    LaserscanMerger _laser_merger;

    dynamic_reconfigure::Server<laserscan_multi_mergerConfig> server;
    dynamic_reconfigure::Server<laserscan_multi_mergerConfig>::CallbackType f;

    f = boost::bind(&LaserscanMerger::reconfigureCallback,&_laser_merger, _1, _2);
	server.setCallback(f);

	ros::spin();

	return 0;
}

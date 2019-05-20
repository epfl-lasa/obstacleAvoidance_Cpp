#include "ros/ros.h"

#include <tf/tf.h>
#include "tf/transform_listener.h"
#include "tf/LinearMath/Quaternion.h"

#include <iostream>
#include <fstream>  // To write data into files
#include <iomanip>
#include <algorithm> // For std::min_element

#include "gazebo_msgs/LinkStates.h"     // For /gazebo/link_states subscriber
#include "geometry_msgs/Twist.h"        // For /cmd_vel            subscriber
#include "sensor_msgs/LaserScan.h"      // For /scan_multi         subscriber
#include "geometry_msgs/PointStamped.h" // For /attractor          subscriber
#include "geometry_msgs/PoseArray.h"    // For /pose_people_map    subscriber
#include "nav_msgs/Odometry.h"          // For /odometry_filtered  subscriber
#include <eigen3/Eigen/Core>

/* Layout of a gazebo_msgs::LinkStates message

string[] name
geometry_msgs/Pose[] pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
geometry_msgs/Twist[] twist
  geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z
*/



class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        ROS_INFO("Starting node");

        // Check if the user has set the simulation variable
        n_.param("/log_data_robot_node/is_simulation", is_simulation, true);

        // Listening to link_states emitted by the Ridgeback for features 1, 2, 3, 4 and 5
        if (is_simulation)
        {
            ROS_INFO("Simulation mode (Gazebo)");
            sub_1 = n_.subscribe("/gazebo/joint_states", 1, &SubscribeAndPublish::callback1_simu, this);
        }
        else
        {
            ROS_INFO("Real experiment mode (Ridgeback)");
            sub_1 = n_.subscribe("/odometry_filtered", 1, &SubscribeAndPublish::callback1_robot, this);
        }

        // Listening to velocity command sent to the robot for features 6, 7 and 8
        sub_2 = n_.subscribe("/cmd_vel", 1, &SubscribeAndPublish::callback2, this);

        // Listening to Hukuyo merged scans for feature 9
        sub_3 = n_.subscribe("/scan_multi", 1, &SubscribeAndPublish::callback3, this);

        // Listening to position of attractor for feature 10
        sub_4 = n_.subscribe("/attractor", 1, &SubscribeAndPublish::callback4, this);

        // Listening to position of detected people for feature 11
        // sub_5 = n_.subscribe("/pose_people_map", 1, &SubscribeAndPublish::callback5, this);

        // Listening to the predicted position of people in (x,y) plane for feature 12
        // sub_6 = ...

        // Log matrix
        log_matrix = Eigen::MatrixXf::Zero(10,5);

        // For callback 1
        time_previous = 0.0;

        // Time start of node
        int time_start = static_cast<int>(std::round(ros::WallTime::now().toSec()));

        // Open log file
        mylog.open("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Logging/data_robot_"+std::to_string(time_start)+".txt", std::ios::out | std::ios_base::app);

        // Check if file has been correctly opened
        if (mylog.fail())
        {
            throw std::ios_base::failure(std::strerror(errno));
        }

        ROS_INFO("data_robot.txt has been opened");

        ROS_INFO("Recording");
    }

    ~SubscribeAndPublish()
    {
        ROS_INFO("Terminating node");
        mylog.close();
        ROS_INFO("data_robot.txt has been closed");
    }

    void callback1_simu(const gazebo_msgs::LinkStates& input)
    {

        // Check if file has been correctly opened
        if (mylog.fail())
        {
            throw std::ios_base::failure(std::strerror(errno));
        }

        // Make sure write fails with exception if something is wrong
        mylog.exceptions(mylog.exceptions() | std::ios::failbit | std::ifstream::badbit);

        // Searching Ridgeback base_link among all links
        for (int i = 0; i < (input.name).size(); i++)
        {
            if (input.name[i]=="ridgeback::base_link") // Selecting Ridgeback info
            {
                // Converting quaternion to roll pitch yaw
                tf::Quaternion q(
                    (input.pose[i]).orientation.x,
                    (input.pose[i]).orientation.y,
                    (input.pose[i]).orientation.z,
                    (input.pose[i]).orientation.w);
                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                // Position of the robot in /map frame
                log_matrix.row(0) << 0, 1.0, round3((input.pose[i]).position.x), round3((input.pose[i]).position.y), round3(static_cast<float>(yaw));

                // Get the position of the robot from /map frame to /odom frame
                transform_and_log(1, (input.pose[i]).position.x, (input.pose[i]).position.y, static_cast<float>(yaw), "map", "odom");

                // Velocity of the robot in /map frame
                log_matrix.row(2) << 0, 3.0, round3((input.twist[i]).linear.x), round3((input.twist[i]).linear.y), round3((input.twist[i]).angular.z);

                // Get the velocity of the robot from /map frame to /odom frame
                transform_and_log(3, (input.twist[i]).linear.x, (input.twist[i]).linear.y, (input.twist[i]).angular.z, "map", "odom");

                // Get the velocity of the robot from /map frame to /base_link frame
                transform_and_log(4, (input.twist[i]).linear.x, (input.twist[i]).linear.y, (input.twist[i]).angular.z, "map", "base_link");
            }
        }

        // Get current time
        ros::Time time_ros = ros::Time::now();
        uint64_t  t_n = time_ros.toNSec();
        float t = static_cast<float>(t_n) * 0.000000001;

        // Write new data
        if ((t-time_previous)>0.05)
        {
            log_matrix.col(0) = t * Eigen::MatrixXf::Ones(log_matrix.rows(),1);
            mylog << log_matrix << "\n";
            time_previous = t;
        }
    }

    void callback1_robot(const nav_msgs::Odometry& input)
    {

        // Check if file has been correctly opened
        if (mylog.fail())
        {
            throw std::ios_base::failure(std::strerror(errno));
        }

        // Make sure write fails with exception if something is wrong
        mylog.exceptions(mylog.exceptions() | std::ios::failbit | std::ifstream::badbit);


        tf::StampedTransform transform_map_to_base;
        // Get the transform between map and base_link
        try
        {
            listener_.lookupTransform("base_link", "map", ros::Time(0), transform_map_to_base);
            //ROS_INFO("Transform is ready");
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
        }

        tf::Quaternion q_map_to_base = transform_map_to_base.getRotation();
        tf::Matrix3x3 m_map_to_base(q_map_to_base);
        double roll, pitch, yaw;
        m_map_to_base.getRPY(roll, pitch, yaw);

        // Position of the robot in /map frame
        log_matrix.row(0) << 0, 1.0, round3(static_cast<float>(transform_map_to_base.getOrigin().x())), round3(static_cast<float>(transform_map_to_base.getOrigin().y())), round3(static_cast<float>(yaw));


        tf::StampedTransform transform_odom_to_base;
        // Get the transform between odom and base_link
        try
        {
            listener_.lookupTransform("base_link", "odom", ros::Time(0), transform_odom_to_base);
            //ROS_INFO("Transform is ready");
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
        }

        tf::Quaternion q_odom_to_base = transform_odom_to_base.getRotation();
        tf::Matrix3x3 m_odom_to_base(q_odom_to_base);
        m_odom_to_base.getRPY(roll, pitch, yaw);

        // Position of the robot in /odom frame
        log_matrix.row(1) << 0, 2.0, round3(static_cast<float>(transform_odom_to_base.getOrigin().x())), round3(static_cast<float>(transform_odom_to_base.getOrigin().y())), round3(static_cast<float>(yaw));

        // Get the velocity of the robot from /base_link frame to /map frame
        transform_and_log(2, input.twist.twist.linear.x, input.twist.twist.linear.y, input.twist.twist.angular.z, "base_link", "map");

        // Get the velocity of the robot from /base_link frame to /odom frame
        transform_and_log(3, input.twist.twist.linear.x, input.twist.twist.linear.y, input.twist.twist.angular.z, "base_link", "odom");

        // Velocity of the robot in /base_link frame
        log_matrix.row(4) << 0, 5.0, round3(input.twist.twist.linear.x), round3(input.twist.twist.linear.y), round3(input.twist.twist.angular.z);


        // Get current time
        ros::Time time_ros = ros::Time::now();
        uint64_t  t_n = time_ros.toNSec();
        float t = static_cast<float>(t_n) * 0.000000001;

        // Write new data
        if ((t-time_previous)>0.05)
        {
            log_matrix.col(0) = t * Eigen::MatrixXf::Ones(log_matrix.rows(),1);
            mylog << log_matrix << "\n";
            time_previous = t;
        }
    }


    void callback2(const geometry_msgs::Twist& input)
    {
        // Get the velocity command of the robot from /base_link frame to /map frame
        transform_and_log(5, input.linear.x, input.linear.y, input.angular.z, "base_link", "map");

        // Get the velocity command of the robot from /base_link frame to /odom frame
        transform_and_log(6, input.linear.x, input.linear.y, input.angular.z, "base_link", "odom");

        // Velocity of the robot in /base_link frame
        log_matrix.row(7) << 0.0, 8.0, round3(input.linear.x), round3(input.linear.y), round3(input.angular.z);
    }

    void callback3(const sensor_msgs::LaserScan& input)
    {
        log_matrix(8,1) = 9.0;
        auto result = std::min_element(std::begin(input.ranges),std::end(input.ranges)); // result is of type __gnu_cxx::__normal_iterator<const float*, std::vector<float> >
        log_matrix(8,2) = *result;
        std::cout << "Minimum distance is " << *result << std::endl;
    }

    void callback4(const geometry_msgs::PointStamped& input)
    {
        log_matrix.row(9) << 0.0, 10.0, input.point.x, input.point.y, 0.0;
    }

    void callback5(const geometry_msgs::PoseArray& input)
    {
        x_people.clear();
        y_people.clear();
        for (int i=0; i<input.poses.size() ; i++)
        {
            geometry_msgs::Pose pose = input.poses[i];
            x_people.push_back(pose.position.x);
            y_people.push_back(pose.position.y);
        }
    }

    float round3(float x) // 3 digits precision
    {
        return std::round(x*1000)*0.001;
    }

    float round4(float x) // 4 digits precision
    {
        return std::round(x*10000)*0.0001;
    }

    void transform_and_log(int i_row, float x, float y, float z, std::string s_from, std::string s_to)
    {
        // Get the transform between s_to and s_from
        try
        {
            listener_.lookupTransform(s_to, s_from, ros::Time(0), transform_);
            //ROS_INFO("Transform is ready");
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
        }

        // Check if the transform between s_to and s_from is ready
        if ( listener_.canTransform(s_to, s_from, ros::Time(0)))
        {
            tf::Vector3 vec3; // Need to use the type tf::Vector3 to make the transformation with tf
            vec3.setX(x);
            vec3.setY(y);
            vec3.setZ(0);
            tf::Stamped<tf::Vector3> vec3_stamped(vec3, ros::Time(0), s_from); // Initial vector in s_from

            tf::Vector3 vec3_transformed;
            tf::Stamped<tf::Vector3> vec3_stamped_transformed(vec3_transformed, ros::Time(0), s_to); // Initial vector in s_to

            listener_.transformVector( s_to, vec3_stamped, vec3_stamped_transformed);

            // Velocity of the robot in /map frame
            log_matrix.row(i_row) << 0.0, static_cast<float>(i_row+1), round3(vec3_stamped_transformed.x()), round3(vec3_stamped_transformed.y()), round3(z);
        }
    }

private:
    ros::NodeHandle n_; // ROS node
    ros::Subscriber sub_1, sub_2, sub_3, sub_4, sub_5, sub_6; // Subscribers
    float time_previous;
    std::vector<float> x_people, y_people;
    Eigen::MatrixXf log_matrix;
    std::ofstream mylog;

    bool is_simulation;

    tf::TransformListener listener_; // To listen to transforms
    tf::StampedTransform transform_; // Transform from base_link to map
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "log_data_robot_node");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    ros::spin();

    return 0;
}


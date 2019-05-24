#include "ros/ros.h"
#include "gazebo_msgs/LinkStates.h"
#include <tf/tf.h>

#include <iostream>
#include <fstream>  // To write data into files
#include <iomanip>

#include "geometry_msgs/PoseArray.h"

// Layout of a gazebo_msgs::LinkStates message
/*
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

        //Topic you want to subscribe
        sub_1 = n_.subscribe("/gazebo/link_states", 1, &SubscribeAndPublish::callback1, this);

        //Topic you want to subscribe
        sub_2 = n_.subscribe("/cmd_vel", 1, &SubscribeAndPublish::callback2, this);

        //Topic you want to subscribe
        sub_3 = n_.subscribe("/pose_people_map", 1, &SubscribeAndPublish::callback3, this);


        std::ofstream mypos, myvel, mycmd;
        mypos.open("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Log_Data/data_position.txt");
	    myvel.open("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Log_Data/data_velocity.txt");
	    mycmd.open("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Log_Data/data_cmd.txt");
        mypos.close();
        myvel.close();
        mycmd.close();

        cmd_x = 0.0;
        cmd_y = 0.0;
        cmd_theta = 0.0;
        time_previous = 0.0;

        ROS_INFO("Recording");
    }

    ~SubscribeAndPublish()
    {
       ROS_INFO("Terminating node");
    }

    void callback1(const gazebo_msgs::LinkStates& input)
    {
        std::ofstream mypos, myvel, mycmd;
	    mypos.open("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Log_Data/data_position.txt", std::ios::out | std::ios_base::app);
	    myvel.open("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Log_Data/data_velocity.txt", std::ios::out | std::ios_base::app);
        mycmd.open("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Log_Data/data_cmd.txt", std::ios::out | std::ios_base::app);

        if (mypos.fail()) {throw std::ios_base::failure(std::strerror(errno));}
	    if (myvel.fail()) {throw std::ios_base::failure(std::strerror(errno));}
        if (mycmd.fail()) {throw std::ios_base::failure(std::strerror(errno));}


	    //make sure write fails with exception if something is wrong
        mypos.exceptions(mypos.exceptions() | std::ios::failbit | std::ifstream::badbit);
	    myvel.exceptions(myvel.exceptions() | std::ios::failbit | std::ifstream::badbit);
        mycmd.exceptions(mycmd.exceptions() | std::ios::failbit | std::ifstream::badbit);

        float t = 0;

        for (int i = 0; i < (input.name).size(); i++)
        {

            if (input.name[i]=="ridgeback::base_link")
            {

                // Convert quaternion to roll pitch yaw
                tf::Quaternion q(
                (input.pose[i]).orientation.x,
                (input.pose[i]).orientation.y,
                (input.pose[i]).orientation.z,
                (input.pose[i]).orientation.w);
                tf::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);

                ros::Time timo = (ros::Time::now());
                //double t = timo.toSec();
                uint64_t  t_n = timo.toNSec();
                t = static_cast<float>(t_n) * 0.000000001;
                //std::cout << t << " and " << static_cast<float>(t_n) * 0.000000001 << std::endl;
                //unsigned long t_nano = (ros::Time::now()).toNsec();
                /*
                mypos << std::setprecision(6) << static_cast<float>(t) << "," << std::setprecision(2) << (input.pose[i]).position.x << "," << std::setprecision(2) << (input.pose[i]).position.y << "," << std::setprecision(2) << yaw << "\n";
                myvel << std::setprecision(6) << static_cast<float>(t) << "," << std::setprecision(2) << (input.twist[i]).linear.x << "," << std::setprecision(2) << (input.twist[i]).linear.y << "," << std::setprecision(2) << (input.twist[i]).angular.z << "\n";
                mycmd << std::setprecision(6) << static_cast<float>(t) << "," << std::setprecision(2) << cmd_x << "," << std::setprecision(2) << cmd_y << "," << std::setprecision(2) << cmd_theta << "\n";
                */
                if (t!=time_previous)
                {
                    mypos << t << "," <<  round3((input.pose[i]).position.x) << "," << round3((input.pose[i]).position.y) << "," << round3(static_cast<float>(yaw)) << "\n";
                    myvel << t << "," <<  round3((input.twist[i]).linear.x) << "," << round3((input.twist[i]).linear.y) << "," << round3((input.twist[i]).angular.z) << "\n";
                    mycmd << t << "," <<  round3(cmd_x) << "," << round3(cmd_y) << "," << round3(cmd_theta) << "\n";
                    time_previous = t;
                }
            }
        }

        mypos.close();
        myvel.close();

        std::ofstream myperson;
        for (int k = 0; k < x_people.size(); k++)
        {
            myperson.open("/home/leziart/catkin_ws/src/process_occupancy_grid/src/Log_Data/data_person" + std::to_string(k) + ".txt", std::ios::out | std::ios_base::app);
            myperson << t << "," << round3(x_people[k]) << "," << round3(y_people[k]) << "\n";
            myperson.close();
        }



    }

    void callback2(const geometry_msgs::Twist& input)
    {
        cmd_x = input.linear.x;
        cmd_y = input.linear.y;
        cmd_theta = input.angular.z;
    }

    void callback3(const geometry_msgs::PoseArray& input)
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

    float round3(float x)
    {
        return std::round(x*1000)*0.001;
    }

        float round4(float x)
    {
        return std::round(x*10000)*0.0001;
    }

private:
    ros::NodeHandle n_;
    ros::Subscriber sub_1, sub_2, sub_3; // To listen to a topic (to be defined)
    float cmd_x, cmd_y, cmd_theta, time_previous;
    std::vector<float> x_people, y_people;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "log_data_node");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    ros::spin();

    return 0;
}


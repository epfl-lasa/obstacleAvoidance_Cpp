Updated the 27/03/2019

- Project_19_03 folder goes into your ~/Documents folder
- Gazebo_worlds folder goes into your ~/Documents folder
- PointCloudFiltering folder goes into your ~/Documents folder
- folders inside ROS_Packages folder go into your ~/catkin_ws/src/ folder

You need to have the Eigen library installed. I used version 3.3.7.  
You can download the latest version with:
* sudo apt-get install libeigen3-dev

Other method (just in case):
* Download the library from http://eigen.tuxfamily.org/index.php?title=Main_Page  
Put library folder (which contains bench, blas, cmake, debug, demos...) into your ~/Documents folder   
Rename the library folder "eigen3" then add the folder to ros library path

Install Ridgeback packages:  
* sudo apt-get install ros-kinetic-ridgeback-simulator ros-kinetic-ridgeback-desktop

Now you have to compile ROS packages with:  
* cd ~/catkin_make  
catkin_make  
source ./devel/setup.bash

Install gmapping with:  
* sudo apt-get install ros-kinetic-slam-gmapping

Install jsk_rviz_plugins to be able to display the velocity command in rviz:
* sudo apt-get install ros-kinetic-jsk-rviz-plugins

---

For each one of the following lines, open a terminal and run the command in it

* roscore

* cd ~/Documents/Gazebo_worlds  
RIDGEBACK_REAR_HOKUYO_LASER="1" roslaunch my_ridgeback.launch

* cd ~/Documents/PointCloudFiltering  
roslaunch filter_for_static.launch

* rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node cloud_in:=/cloud_for_static

* rosrun gmapping slam_gmapping scan:=/scan _delta:=0.3 _map_update_interval:=1.0

* rosrun process_occupancy_grid process_occupancy_grid_node

* rosrun process_occupancy_grid cmd_vel_listener 

* rosrun rviz rviz ~/Documents/Gazebo_worlds/config_rviz.rviz

---

In rviz (already done in the config_rviz.rviz file):  
* Set the global fixed frame to "map"
* Add a Map object and set it to the "map" topic
* Add a RobotModel object, the Ridgeback should appear
* Add PointCloud2 object that displays "point_cloud_for_static" or "velodyne_points"
* Add Laserscan object that displays "scan"

---

Once everything has been set, open a new terminal then run:

* rostopic pub /test geometry_msgs/Twist -r 10 -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

This topic triggers the callback function that computes the velocity command. It is set at 10 Hz but you can change it if you want. For instance "-r 1" to get 1 Hz.

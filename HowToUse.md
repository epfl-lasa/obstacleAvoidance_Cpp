Updated 06/05/2019

- Content of Folders_Documents folder goes into your ~/Documents folder
- Copy-paste content of /catkin_ws/src/ into your ~/catkin_ws/src/ folder

You need to have the Eigen library installed. I used version 3.3.7.  
You can download the latest version with:
* sudo apt-get install libeigen3-dev

Other method (not needed if the first method works):
* Download the library from http://eigen.tuxfamily.org/index.php?title=Main_Page  
Put library folder (which contains bench, blas, cmake, debug, demos...) into your ~/Documents folder   
Rename the library folder "eigen3" then add the folder to ros library path

Install Ridgeback packages:  
* sudo apt-get install ros-kinetic-ridgeback-simulator ros-kinetic-ridgeback-desktop

Now you have to compile ROS packages with:  
* cd ~/catkin_ws 
catkin_make  
source ./devel/setup.bash

Install gmapping with:  
* sudo apt-get install ros-kinetic-slam-gmapping

Install jsk_rviz_plugins to be able to display the velocity command in rviz (not compulsory):
* sudo apt-get install ros-kinetic-jsk-rviz-plugins

---

You need the following models to launch the world: (https://bitbucket.org/osrf/gazebo_models/src/e6d645674e8a99cb7955ce3b9a1fbe26c58c41f2/)
* bookshelf
* collapsed_industrial
* dumpster
* fountain
* ground_plane
* person_standing
* sun
* willowgarage
If you use "hg clone https://bitbucket.org/osrf/gazebo_models" you get all the models but apparently you select which folders you want with a python script or with "mercurial"

---

For each one of the following lines, open a terminal and run the command in it

* roscore

* roslaunch worlds_and_launchers my_ridgeback.launch

* roslaunch worlds_and_launchers filter_for_static.launch

* rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node cloud_in:=/cloud_for_static

* rosrun gmapping slam_gmapping scan:=/scan _delta:=0.2 _map_update_interval:=1.0

* rosrun process_occupancy_grid process_occupancy_grid_node

* rosrun process_occupancy_grid cmd_vel_listener 

* rosrun rviz rviz ~/catkin_ws/src/worlds_and_launchers/rviz/config_rviz.rviz

---

In rviz (already done if you use the config_rviz.rviz file):  
* Set the global fixed frame to "map"
* Add a Map object and set it to the "map" topic
* Add a RobotModel object, the Ridgeback should appear
* Add PointCloud2 object that displays "point_cloud_for_static" or "velodyne_points"
* Add Laserscan object that displays "scan"
* Add TwistStamped object that displays "cmd_vel_stamped" (need jsk_rviz_plugins)

---

Once everything has been set, open a new terminal then run:

* rostopic pub /test geometry_msgs/Twist -r 10 -- '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

This topic triggers the callback function that computes the velocity command. It is set at 10 Hz but you can change it if you want. For instance "-r 1" to get 1 Hz.

---

* d415.stl and d435.dae have been removed from realsense2_camera/meshes folder to save storage space
* Qolo_T_CB_Single.stl has been removed from process_depth_img/model_Qolo/meshes folder to save storage space

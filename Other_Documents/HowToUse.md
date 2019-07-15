Updated 07/05/2019

- Content of Folders_Documents folder goes into your ~/Documents folder

If you only want to run the simulation, copy and paste content of /catkin_ws/src_for_simulation/ into your ~/catkin_ws/src/ folder

You need to have the Eigen library installed. I used version 3.3.7.  
You can download the latest version with:
* sudo apt-get install libeigen3-dev

Other method (not needed if the first method works):
* Download the library from http://eigen.tuxfamily.org/index.php?title=Main_Page  
Put library folder (which contains bench, blas, cmake, debug, demos...) into your ~/Documents folder   
Rename the library folder "eigen3" then add the folder to ros library path

Install Ridgeback packages:  
* sudo apt-get install ros-kinetic-ridgeback-simulator ros-kinetic-ridgeback-desktop

Install gmapping with:  
* sudo apt-get install ros-kinetic-slam-gmapping

Install jsk_rviz_plugins to be able to display the velocity command in rviz (not compulsory):
* sudo apt-get install ros-kinetic-jsk-rviz-plugins

Now you have to compile ROS packages with:  
* cd ~/catkin_ws 
catkin_make  
source ./devel/setup.bash

---

## Needed models

You need the following models to launch the world: (https://bitbucket.org/osrf/gazebo_models/src/e6d645674e8a99cb7955ce3b9a1fbe26c58c41f2/)
* bookshelf
* collapsed_industrial
* dumpster
* fountain
* ground_plane
* person_standing
* sun
* willowgarage

If you launch Gazebo without those models it should download them by itself but I did not tried.

If you use "hg clone https://bitbucket.org/osrf/gazebo_models" you get all the models but apparently you can select which folders you want with a python script or with "mercurial".

---

## Command lines

If all obstacles are static, use each one of the following lines, open a terminal and run the command in it

* roscore

* roslaunch worlds_and_launchers my_ridgeback.launch

* roslaunch worlds_and_launchers filter_for_static.launch

* rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node cloud_in:=/cloud_for_static _range_max:=20.0

* rosrun gmapping slam_gmapping scan:=/scan _delta:=0.15 _map_update_interval:=1.0

* rosrun process_occupancy_grid process_occupancy_grid_node

* rosrun process_occupancy_grid cmd_vel_listener 

* rosrun rviz rviz ~/catkin_ws/src/worlds_and_launchers/rviz/config_rviz.rviz

If there are moving people, they need to be removed from the velodyne point cloud.

* roscore

* roslaunch worlds_and_launchers my_ridgeback.launch

* roslaunch worlds_and_launchers filter_for_static.launch

* rosrun remove_people_pcl remove_people_pcl_node

* rosrun pointcloud_to_laserscan pointcloud_to_laserscan_node cloud_in:=/cloud_without_people _range_max:=20.0

* rosrun gmapping slam_gmapping scan:=/scan _delta:=0.15 _map_update_interval:=1.0

* rosrun process_occupancy_grid process_occupancy_grid_node

* rosrun process_occupancy_grid vector_to_posearray_node

* rosrun process_occupancy_grid cmd_vel_listener 

* rosrun rviz rviz ~/catkin_ws/src/worlds_and_launchers/rviz/config_rviz.rviz

Basically `remove_people_pcl_node` removes people from /cloud_for_static and outputs /cloud_without_people. `vector_to_posearray_node` makes the link between gazebo and ROS by grouping the position of all people (vectors) into a single PoseArray. This PoseArray is used both by `remove_people_pcl_node` and `process_occupancy_grid_node`.

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

### Logging

There are two logging systems. The first one logs information about the robot. If it is a simulation then set `_is_simulation:=true` to make the system listen to Gazebo. Set `_is_simulation:=false` if it is a real experiment with the Ridgeback platform. Recorded data is in process_occupancy_grid/src/Logging/data_robot_XXX.txt

* rosrun process_occupancy_grid log_data_robot_node _is_simulation:=true
* rosrun process_occupancy_grid log_data_robot_node _is_simulation:=false

The second logging system records information about the obstacle avoidance algorithm (detected obstacles, boundaries, generated velocities). To use it set `bool logging_enabled = true;` in ObstacleReconstruction.cpp then `catkin_make`. Recorded data is in process_occupancy_grid/src/Logging/data_obstacles_XXX.txt

There is also a node to log sensors data /velodyne_points, /scan and /camera/color/image_raw. Set to true if you want to record one of them, default recording frequency is 10 Hz. Recorded data is in process_occupancy_grid/src/Logging/data_cloud_XXX.txt or data_laserscan_XXX.txt or data_image_XXX.txt

* rosrun process_occupancy_grid log_data_sensors_node _log_cloud:=false _freq_cloud:=10 _log_laserscan:=false _freq_laserscan:=10 _log_image:=false _freq_image:=10


---

## Removed models

* d415.stl and d435.dae have been removed from realsense2_camera/meshes folder to save storage space
* Qolo_T_CB_Single.stl has been removed from process_depth_img/model_Qolo/meshes folder to save storage space

---

## Custom use

If you want to disable the graphical display of gazebo, in /worlds_and_launchers/launch/my_ridgeback.launch, change `name="gui" default="true"` to `name="gui" default="false"`

If you want to load another world, comment the current one in /worlds_and_launchers/launch/my_ridgeback.launch with `<!-- [...] -->` and uncomment the one you want.

If you want to modify some stuff for the world then look into the /worlds_and_launchers/worlds folder.

---

## To use the Realsense camera with people detection:

* roscore

* roslaunch movidius_ncs_launch ncs_camera.launch cnn_type:=mobilenetssd camera:=realsense

* cd ~/catkin_ws/src/ros_intel_movidius_ncs/movidius_ncs_launch/launch

* roslaunch my_ncs_stream_detection_example.launch camera_topic:="/camera/color/image_raw"

* rosrun kalman_bounding_boxes kalman_bounding_boxes_node 

* rosrun process_depth_img process_depth_img_node 


#include "ros/ros.h"
#include <tf/tf.h>
#include "tf/transform_listener.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Scalar.h"
#include "tf/LinearMath/Vector3.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3Stamped.h"

//#include "obstacle_avoidance/GetObstacles.h"
#include <cstdlib>

// Eigen library and the header that contains my functions
#include <eigen3/Eigen/Core>
#include "ObstacleReconstruction.h"
#include "ObstacleAvoidance.h"

#include <fstream>  // To write data into files

typedef Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic> MatrixXi8; // Dynamic Eigen matrix with type int8_t since OccupancyGrid contains int8_t values
typedef Eigen::Matrix<int8_t, 1, Eigen::Dynamic> MatrixXi8_layer;
typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> myMap;

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/test", 1, &SubscribeAndPublish::callback, this);

    //Another topic you want to subscribe
    sub_people = n_.subscribe("/pose_people", 2, &SubscribeAndPublish::callback_for_people, this);

    // Client of dynamic_map service of gmapping
    client_map_ = n_.serviceClient<nav_msgs::GetMap>("dynamic_map");

    // Transform listener
    tf::TransformListener transform_(ros::Duration(10.0));

  }

  void callback(const geometry_msgs::Twist& input)//const nav_msgs::OccupancyGrid& input)
  {

    // Call gmapping service
    nav_msgs::GetMap srv_map;
    if (client_map_.call(srv_map))
    {
        ROS_INFO("Dynamic map received"); //map is in srv.response.map
    }
    else
    {
       ROS_ERROR("Failed to call dynamic map service");
    }

    // Retrieving OccupancyGrid data that has been received
    std::vector<int8_t> test_vector = (srv_map.response.map.data); // get map into a std container
    // occupancy grid is now stored in the vector (1 dimension) with type int8_t

    // Convert std container to type int
    std::vector<int> int_vector(test_vector.begin(), test_vector.end());

    // Get std container into an Eigen matrix with the right width and height
    Eigen::Map<Eigen::MatrixXi> eig_test( int_vector.data(), srv_map.response.map.info.height, srv_map.response.map.info.width);
    //std::cout << "Size after cast: " << eig_test.rows() << " " << eig_test.cols() << std::endl;


    float size_cell = 0.3;

    // Once the occupancy grid has been reshaped, c++ algorithms can be used

    // Expand obstacles to get a security margin
    Eigen::MatrixXi eig_expanded = expand_occupancy_grid( eig_test, static_cast<int>(std::floor(1/size_cell)));

    

    // Detect expanded obstacles
    std::vector<Border> storage;
    storage = detect_borders( eig_expanded );
   
    // Storage temp
    Eigen::MatrixXi eig_blob = expand_occupancy_grid( eig_test, static_cast<int>(std::floor(1/size_cell)));
    std::vector<Blob> storage_blobs;
    storage_blobs = detect_blobs( eig_blob);

    /*std::ofstream myblobs;
    std::cout << storage_blobs.size() << " blobs have been detected." << std::endl;
    for (int iter=0; iter < storage_blobs.size(); iter++)
    { 
        myblobs.open("./gazebo_obstacle_" + std::to_string(iter) + ".txt"); //each starting point has its own file
        Blob blob = storage_blobs[iter];
	for (int k=0; k < blob.rows(); k++)
        {
             myblobs << "obst" << iter << ".row(" << k << ") << " << blob(k,0) << " , " << blob(k,1) << "; \n"; 
        }
        myblobs.close();
    }*/
    
    /*for (int iter=0; iter < storage.size(); iter++)
    {
        std::cout << "Obstacle " << iter << ":"<< std::endl;
        std::cout << storage[iter] << std::endl;
    }*/
    std::cout << storage.size() << " obstacles have been detected." << std::endl;

    /*eig_expanded(315,312) = 42;
    eig_expanded(319,343) = 42;
    eig_expanded(340,334) = 42;
    eig_expanded(349,329) = 42;
    std::cout << eig_expanded.block(340,310,41,41) << std::endl;*/

    /*std::ofstream mypoints;
    mypoints.open("expanded_map.txt");
    mypoints << eig_expanded; 
    mypoints.close();*/

    // NO NEED SINCE THE MAP IS USED AS THE REFERENCE FRAME
    // Retrieving Resolution and Pose of the map in the world frame
    float resolution = srv_map.response.map.info.resolution;
    float x_pose = srv_map.response.map.info.origin.position.x;
    float y_pose = srv_map.response.map.info.origin.position.y;
    ROS_INFO("Map origin: %f %f", x_pose, y_pose);
    /*tfScalar roll, pitch, yaw;
    tf::Matrix3x3 mat(srv_map.response.map.info.origin.orientation);
    mat.getRPY(roll, pitch, yaw);
    float phi_pose = yaw;*/

    // Get the transform between /map and /base_link (to get the position of the robot in the map)
    try{
      //listener_.waitForTransform("/base_link", "/map", ros::Time(0), ros::Duration(10.0) );
      listener_.lookupTransform("map", "base_link", ros::Time(0), transform_);
      ROS_INFO("Transform is ready");
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
    }
    std::cout << "Translation X: " << transform_.getOrigin().getX() << std::endl;
    std::cout << "Translation Y: " << transform_.getOrigin().getY() << std::endl;
    // State of the robot
    tfScalar yaw, pitch, roll;
    tf::Matrix3x3 mat(transform_.getRotation());
    mat.getRPY(roll, pitch, yaw);

    State state_robot; state_robot << (transform_.getOrigin().getX() - x_pose)/ size_cell,
                                      (transform_.getOrigin().getY() - y_pose)/ size_cell,
                                      yaw;
    ROS_INFO("Robot %f %f %f", state_robot(0,0), state_robot(1,0), state_robot(2,0));

    // Map display centered on the robot
    /*std::cout << eig_test.block(static_cast<int>(std::floor(x_pose - transform_.getOrigin().getX()/size_cell)-20), static_cast<int>(std::floor(y_pose - transform_.getOrigin().getY()/size_cell)-20),41,41) << std::endl;*/

    // Set state of the attractor
    State state_attractor; state_attractor << 380,331,0;

    // Compute velocity command based on the detected obstacles
    //State next_eps = next_step_several_obstacles_border( state_robot, state_attractor, storage);


    Eigen::Matrix<float, 1, 2> robot; robot << state_robot(0,0), state_robot(1,0);
    
    Eigen::MatrixXf closest(1,6); // Get closest cell for the robot
    Eigen::Matrix<float, 6, 1> gamma_norm_proj; // Get gamma distance + normal vector + point projected on border for the robot

    /*closest = find_closest_point(robot, storage[0]);
    gamma_norm_proj = gamma_normal_projection( robot, closest);
    
    int closest_i = 0;
    int closest_dist = gamma_norm_proj(0,0);

    for (int iter=1; iter< storage.size(); iter++)
    {
         closest = find_closest_point(robot, storage[iter]);
         gamma_norm_proj = gamma_normal_projection( robot, closest);
         if (gamma_norm_proj(0,0) < closest_dist)
         {
              closest_i = iter;
              closest_dist = gamma_norm_proj(0,0);
         }
    }

    State next_eps = next_step_special( state_robot, state_attractor, storage[closest_i]); // only consider closest obstacle*/

    State next_eps = next_step_special_weighted( state_robot, state_attractor, storage); // consider all obstacles
    next_eps(2,0) = std::atan2(next_eps(1,0),next_eps(0,0)) - state_robot(2,0);
    next_eps = speed_limiter(next_eps);
    ROS_INFO("VelCmd in map frame: %f %f %f", next_eps(0,0), next_eps(1,0), next_eps(2,0));
    //next_eps << 0,0,0;

    // We have to convert the velocity command from the map frame to the base_link frame
    tf::Vector3 vec3;
    vec3.setX(next_eps(0,0));
    vec3.setY(next_eps(1,0));
    vec3.setZ(0);
    tf::Vector3 vec3_transformed;

    tf::Stamped<tf::Vector3> vec3_stamped(vec3, ros::Time(0), "map");
    
    tf::Stamped<tf::Vector3> vec3_stamped_transformed(vec3_transformed, ros::Time(0), "base_link");

    listener_.transformVector( "base_link", vec3_stamped, vec3_stamped_transformed);
    ROS_INFO("VelCmd in base_link frame: %f %f %f", vec3_stamped_transformed.x(), vec3_stamped_transformed.y(), next_eps(2,0));

    // Creating velocity message
    float scaling = 4;
    geometry_msgs::Twist output;
    output.linear.x = scaling * vec3_stamped_transformed.x();
    output.linear.y = scaling * vec3_stamped_transformed.y();
    output.linear.z = 0.0;
    output.angular.x = 0.0;
    output.angular.y = 0.0;
    output.angular.z = scaling * next_eps(2,0);

    //ROS_INFO("VelCmd : %f %f %f", next_eps(0,0), next_eps(1,0), next_eps(2,0));
    pub_.publish(output);
  }

void callback_for_people(const geometry_msgs::PoseArray& people)
{
    Eigen::MatrixXf temp_storage;

    for (int i=0; i<people.poses.size(); i++)
    {
        temp_storage.resize(10, temp_storage.cols()+1); // add a column to store the new detected person

        // TODO: test implementation of tranformPose once it is done
        geometry_msgs::PoseStamped pose_person;
        pose_person.header = people.header;
        pose_person.pose = people.poses[i];
        geometry_msgs::PoseStamped pose_person_in_map;
        pose_person_in_map.header = people.header;
        //listener.tranformPose("map", pose_person, pose_person_in_map)

        float personX = pose_person.pose.position.x;
        float personY = pose_person.pose.position.y;
        float personZ = pose_person.pose.position.z;

        // Check if it's (X,Z) and not (Z,X) or (X,Y)...
        // [x_c, y_c, phi, a1, a2, p1, p2, v_x, v_y, w_rot]
        temp_storage.col(i) << personX, personZ, 0, 1, 1, 1, 1, 0, 0, 0; // circle centered on the person

    }
    detected_people = temp_storage;
    std::cout << "Detected people :" << std::endl << detected_people.transpose() << std::endl;
}

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;  // To publish the velocity command
  ros::Subscriber sub_; // To listen to a topic (to be defined)
  ros::Subscriber sub_people; // To listen to a topic (to be defined)
  tf::TransformListener listener_; // To listen to transforms
  tf::StampedTransform transform_; // Transform from map to base_link
  ros::ServiceClient client_map_;  // To call dynamic_map service of gmapping
  MatrixXi8 eig_map_;
  bool flag_map;
  Eigen::MatrixXf detected_people;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "process_occupancy_grid_node");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}

// Tutorial ROS : Writing a tf listener (C++)
// Class found in ros answers : "Publishing to a topic via subscriber callback function"
// How to initialize a UInt8MultiArray message

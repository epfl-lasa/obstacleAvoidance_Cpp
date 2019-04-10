#include "ros/ros.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "tf/transform_listener.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Scalar.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include <cstdlib>

// For synchronization of subscribed topics
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Subscribed messages
#include "object_msgs/ObjectsInBoxes.h"

// Eigen library and the header that contains my functions
#include <eigen3/Eigen/Core>

#include <fstream>  // To write data into files

using namespace message_filters;

bool flag_write = true;
std::ofstream myfile;


/*
class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    //Topic you want to subscribe
    //sub_bounding = n_.subscribe("/camera/depth_registered/points", 1, &SubscribeAndPublish::callback, this);

    //Topic you want to subscribe
    sub_bounding(n_, "/darknet_ros/bounding_boxes", 1);

    //Topic you want to subscribe
    sub_depth(n_, "/camera/depth_registered/points", 1);

    // Synchronizer
    sync(sub_bounding, sub_depth, 10);
    sync.registerCallback(boost::bind(&SubscribeAndPublish::callback, _1, _2));

    // Transform listener
    tf::TransformListener transform_(ros::Duration(10.0));
  }

  void callback(const darknet_ros_msgs::BoundingBoxes& boxes, const sensor_msgs::PointCloud2& pc2)
  {
    // Retrieving PointCloud2 data
    std::vector<uint8_t> test_vector = pc2.data;
    // PointCloud2 is now stored in the vector (1 dimension) with type int8_t

    // Position of the pixel in the picture
    int x = 320;
    int y = 240;

    // Position of the information (X,Y,Z) in the world frame
    // See the documentation of sensor_msgs/PointCloud2 to understand the offsets and steps
    int arrayPosition = y * pc2.row_step + x * pc2.point_step;
    // Array Position is the start of a 32 bytes long sequence that contains X, Y, Z and RGB data
    // about the pixel at position (x,y) in the picture
    int arrayPosX = arrayPosition + pc2.fields[0].offset;
    int arrayPosY = arrayPosition + pc2.fields[1].offset;
    int arrayPosZ = arrayPosition + pc2.fields[2].offset;
    float X_pos = 0; float Y_pos = 0; float Z_pos = 0;
    memcpy(&X_pos, &pc2.data[arrayPosX], sizeof(float));
    memcpy(&Y_pos, &pc2.data[arrayPosY], sizeof(float));
    memcpy(&Z_pos, &pc2.data[arrayPosZ], sizeof(float));

    std::cout << "(X,Y,Z) = (" << X_pos << "," << Y_pos << "," << Z_pos << ")" << std::endl;

    //pub_.publish(output);
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;  // To publish the velocity command
  //ros::Subscriber sub_bounding; // Subscriber to bounding boxes
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_bounding; // Subscriber to bounding boxes
  message_filters::Subscriber<sensor::msgs::PointCloud2> sub_depth;          // Subscriber to depth image
  TimeSynchronizer<darknet_ros_msgs::BoundingBoxes,sensor::msgs::PointCloud2> sync; // Synchronizer for the three topics
  tf::TransformListener listener_; // To listen to transforms
  tf::StampedTransform transform_; // Transform from map to base_link


};//End of class SubscribeAndPublish
*/


ros::Publisher pub_;

void callback(const object_msgs::ObjectsInBoxes::ConstPtr& boxes, const sensor_msgs::PointCloud2::ConstPtr& pc2)
{
    ROS_INFO("Callback triggered");
    // PointCloud2 3D data is stored in the "data" field of pc2
    // It is stored as a long 1-dimensional array with type int8_t


    // Number of detected people in the field of view of the camera
    std::cout << "Nb boxes: " << (boxes->objects_vector).size() << std::endl;

    geometry_msgs::PoseArray people;
    people.header.stamp = ros::Time::now();
    people.header.frame_id = "camera_color_optical_frame";
    geometry_msgs::Pose pose_person;
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    pose_person.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0); 

    for (int i_box=0; i_box < (boxes->objects_vector).size(); i_box++)
    {

        // Position of the pixel at the center of the bouding box (in the picture)
        int x = static_cast<int>(std::floor( (boxes->objects_vector[0]).roi.x_offset + (boxes->objects_vector[0]).roi.width/2));
        int y = static_cast<int>(std::floor( (boxes->objects_vector[0]).roi.y_offset + (boxes->objects_vector[0]).roi.height /2));
        ROS_INFO("Center of box at position (%i,%i)", x, y);

	if (flag_write)
	{
		flag_write = false;
		myfile.open("data_process_depth_img.txt");
		for (int i= (boxes->objects_vector[0]).roi.x_offset ; i < ((boxes->objects_vector[0]).roi.x_offset + (boxes->objects_vector[0]).roi.width); i++)
		{
			for (int j= (boxes->objects_vector[0]).roi.y_offset ; j < ((boxes->objects_vector[0]).roi.y_offset + (boxes->objects_vector[0]).roi.height); j++)
			{
				float X_tempo = 0;
				float Y_tempo = 0;
				float Z_tempo = 0;
				int arrayPoso = (j) * pc2->row_step + (i) * pc2->point_step;	
				// about the pixel at position (x,y) in the picture
				int arrayPosoX = arrayPoso + pc2->fields[0].offset;
				int arrayPosoY = arrayPoso + pc2->fields[1].offset;
				int arrayPosoZ = arrayPoso + pc2->fields[2].offset;
				memcpy(&X_tempo, &pc2->data[arrayPosoX], sizeof(float));
				memcpy(&Y_tempo, &pc2->data[arrayPosoY], sizeof(float));
				memcpy(&Z_tempo, &pc2->data[arrayPosoZ], sizeof(float));
				
				myfile << i << "," << j << "," << Z_tempo << "\n";	
			}
		}
		myfile.close();		
	}


        // Mean of the depth over 9 points
        float X_pos = 0 ;
        float Y_pos = 0 ;
        float Z_pos = 0 ;
        float X_temp = 0;
        float Y_temp = 0;
        float Z_temp = 0;
        const int pixel_diff = 3;
        int num_points = 9;
        for (int i=-pixel_diff; i <=pixel_diff; i += pixel_diff)
        {
            for (int j=-pixel_diff; j <=pixel_diff; j += pixel_diff)
            {
                // Position of the information (X,Y,Z) in the world frame
                // See the documentation of sensor_msgs/PointCloud2 to understand the offsets and steps
                int arrayPosition = (y+j) * pc2->row_step + (x+i) * pc2->point_step;
                // Array Position is the start of a 32 bytes long sequence that contains X, Y, Z and RGB data        
                
                // about the pixel at position (x,y) in the picture
                int arrayPosX = arrayPosition + pc2->fields[0].offset;
                int arrayPosY = arrayPosition + pc2->fields[1].offset;
                int arrayPosZ = arrayPosition + pc2->fields[2].offset;
                memcpy(&X_temp, &pc2->data[arrayPosX], sizeof(float));
                memcpy(&Y_temp, &pc2->data[arrayPosY], sizeof(float));
                memcpy(&Z_temp, &pc2->data[arrayPosZ], sizeof(float));
                
                // Need to check if X,Y,Z are not NaN in case the considered pixel was not considered for this
                // frame (it can happen, the resulting depth cloud has lots of holes)
                if (!(X_pos != X_pos)) // (f != f) is true only when f is NaN
                {
                    X_pos += X_temp;
                    Y_pos += Y_temp;
                    Z_pos += Z_temp;
                }
                else
                {
                    num_points -= 1; // the point is not considered
                }
            }
        }
        

        X_pos /= num_points;
        Y_pos /= num_points;
        Z_pos /= num_points; // dividing by the number of points

        std::cout << "(X,Y,Z) = (" << X_pos << "," << Y_pos << "," << Z_pos << ")" << std::endl;
        
        pose_person.position.x = X_pos;
        pose_person.position.y = Y_pos;
        pose_person.position.z = Z_pos;
        people.poses.push_back(pose_person);
    }
    // TODO: Fill a PoseArray with all detected people and broadcast it

    pub_.publish(people);
}

void tfCallback(const sensor_msgs::PointCloud2::ConstPtr& pc2)
{
    // Initialization of the tranform publisher between the camera frame and the world frame
    // Parent: "map"  Child: "camera_color_optical_frame"
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);

    // Sending transform between camera frame and world frame
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "camera_link"));
}

int main(int argc, char **argv)
{
    ROS_INFO("Starting node");
    
    //Initiate ROS
    ros::init(argc, argv, "process_depth_img_node");

    //Create an object of class SubscribeAndPublish that will take care of everything
    //SubscribeAndPublish SAPObject;

    ros::NodeHandle n_;
    ros::Duration(3).sleep(); 

    //ros::Subscriber sub_bounding; // Subscriber to bounding boxes

    message_filters::Subscriber<object_msgs::ObjectsInBoxes> sub_bounding(n_, "/filtered_boxes", 1); // Subscriber to bounding boxes

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_depth(n_, "/camera/depth_registered/points", 1);       // Subscriber to depth point cloud

    typedef sync_policies::ApproximateTime<object_msgs::ObjectsInBoxes, sensor_msgs::PointCloud2> MySyncPolicy;  // Synchronization policy

    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(20),sub_bounding, sub_depth); // Synchronizer for the two topics

    sync.registerCallback(boost::bind(&callback, _1, _2)); // Link synchronizer with callback

    ros::Subscriber sub_for_tf = n_.subscribe("/camera/depth_registered/points", 1, tfCallback);

    pub_ = n_.advertise<geometry_msgs::PoseArray>("/pose_people",1);

    ros::spin();

    return 0;
}



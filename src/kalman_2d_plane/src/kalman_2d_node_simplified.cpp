#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/PointCloud2.h"
#include "KalmanFilter.h"

// Eigen library and the header that contains my functions
#include <eigen3/Eigen/Core>

#include <fstream>  // To write data into files
#include <limits>

#include <tf/tf.h>
#include <tf/transform_listener.h>

typedef Eigen::Matrix<int8_t, 1, Eigen::Dynamic> MatrixXi8_layer;
typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> myMap;

using namespace std;

const int limit_age = 25;
const int born_age = limit_age - 4;
const int limit_dist = 1.5;
const bool verbose = true;

/* Initialization */
int n_states = 4;
float x_box, y_box, x_pred, y_pred;
float sigma_1, sigma_2, sigma_vx, sigma_vy, sigma_0, h, vx_box, vy_box;
Eigen::VectorXf Z;
Eigen::MatrixXf A(n_states, n_states);
Eigen::MatrixXf H(n_states, n_states);
Eigen::MatrixXf Q(n_states, n_states);
Eigen::MatrixXf R(n_states, n_states);
Eigen::VectorXf X0(n_states);
Eigen::MatrixXf P0(n_states, n_states);

void init_parameters_kalman()
{
    /* Set the parameters (standard deviations) */
    sigma_1 = 0.01;//0.25;
    sigma_2 = 0.01;//0.25;
    sigma_vx = 0.01;//0.3;
    sigma_vy = 0.01;//0.3;
    sigma_0 = 0.1;//0.5;
    h = 1.0f;//0.125f;

    /* Set Matrix and Vector for Kalman Filter: */

    A << 1, 0, h, 0,
         0, 1, 0, h,
         0, 0, 1, 0,
         0, 0, 0, 1;


    H << 1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1;


        Q << std::pow(h*sigma_vx,2),          0,        0,        0,
                                  0, std::pow(h*sigma_vy,2),        0,        0,
                                  0,          0, std::pow(sigma_vx,2),        0,
                                  0,          0,        0, std::pow(sigma_vy,2);

        R << std::pow(sigma_1,2),       0,         0,         0,
                               0, std::pow(sigma_2,2),        0,         0,
                               0,       0, std::pow(sigma_1/h,2),        0,
                               0,       0,         0,  std::pow(sigma_2/h,2);

        X0 << 0, 0, 0, 0; // default position

        P0 << 0, 0,       0,       0,
              0, 0,       0,       0,
              0, 0, std::pow(sigma_0,2),       0,
              0, 0,       0, std::pow(sigma_0,2);

}

// DECLARATION STUCTURE TRACKED_PERSON
struct tracked_person
{
    int age; // number of steps since this person was updated
    float info[2]; // [x, y] position of the person in the 2D horizontal plane
    KalmanFilter filter_person;
    float x_prev, y_prev;
    bool has_been_updated = false;

    tracked_person(float x, float y);
};

tracked_person::tracked_person(float x, float y)
{
    info[0] = x;
    info[1] = y;
    x_prev = x;
    y_prev = y;
    age = born_age;
    KalmanFilter filter_temp(n_states,0);
    filter_person = filter_temp;
    /* Initialize the Filter*/
    filter_person.setFixed(A, H, Q, R);
    X0 << x,y,0.0,0.0;
    filter_person.setInitial(X0, P0);
}
// END STUCTURE TRACKED_PERSON

/* Structure of a geometry_msgs::PoseArray message
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose[] poses
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
*/

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        // Initialization of Kalman parameters
        init_parameters_kalman();

        // Publishing people's position inside and outside the FOV of Realsense cameras
        pub_ = n_.advertise<geometry_msgs::PoseArray>("/pose_people_velodyne_filtered", 5);

        // Publishing people's position inside and outside the FOV of Realsense cameras
        pub_in_map_ = n_.advertise<geometry_msgs::PoseArray>("/pose_people_map_filtered", 5);

        // Subscribing to people's position detected by Realsense cameras
        sub_ = n_.subscribe("/pose_people_velodyne", 1, &SubscribeAndPublish::callback, this);

        // Subscribing to Velodyne point cloud
        sub_pcl = n_.subscribe("/cloud_for_static", 1, &SubscribeAndPublish::callback_pcl, this);

        /* Set the parameters (standard deviations) */
        Z = Eigen::VectorXf(n_states);
        KalmanFilter filter_temp(n_states,0);
        filter1 = filter_temp;

        /* Initialize the Filter*/
        filter1.setFixed(A, H, Q, R);
        filter1.setInitial(X0, P0);

        /* Time start of node */
        time_start_clock = ros::Time::now().toSec();

        ROS_INFO("Kalman filter for 2D horizontal plane has been initialized.");
    }

    void callback(const geometry_msgs::PoseArray& input)
    {

        if (verbose) {ROS_INFO("Receiving raw bounding boxes");}

        geometry_msgs::PoseArray output = input;
        discarded_poses.clear();
        // REMOVE OLD TRACKERS //

        for (int i=0; i < tracked_people.size(); i++)
        {
            (tracked_people[i]).has_been_updated = false;
            //(tracked_people[i]).age += 1;
            if (verbose) {ROS_INFO("Filter %i is now %i steps old", i, (tracked_people[i]).age);}
            if (((tracked_people[i]).age) > limit_age)
            {
                tracked_people.erase(tracked_people.begin() + i); // remove the person from the list
                only_trackers.erase(only_trackers.begin() + i);
                if (verbose) {ROS_INFO("Filter %i deleted because it is too old", i);}
                i -= 1; // go on step back since an element has been removed
            }
        }
 
        // REMOVE TRACKER TOO FAR AWAY OR NaN
        for (int i=0; i < tracked_people.size(); i++)
        {
	    float x_tracker = (((tracked_people[i]).filter_person).X)[0];
            float y_tracker = (((tracked_people[i]).filter_person).X)[1];            

            if ((std::sqrt(std::pow(x_tracker,2)+std::pow(y_tracker,2))>5)||(std::isnan(x_tracker))||(std::isnan(y_tracker)))
            {
                tracked_people.erase(tracked_people.begin() + i); // remove the person from the list
                only_trackers.erase(only_trackers.begin() + i);
                if (verbose) {ROS_INFO("Filter %i deleted because it is too far away or NaN", i);}
                i -= 1; // go on step back since an element has been removed   
            }
        }


        // PAIR DETECTED PEOPLE WITH THE LIST OF TRACKED PEOPLE
        // For each detected people
        // - find closest tracked people
        // - if two detected people share the same tracked people, a copy is created for the second one
        // - if a tracked people has no associated tracked people (there are all out of the limit range), a new tracked_people is created
        if ((input.poses).size() != 0)
        {
            std::vector<int> index_match((input.poses).size(), std::numeric_limits<int>::max());
            /*for (int i=0; i < (input.poses).size(); i++)
            {
                std::cout << index_match[i] << std::endl;
            }*/
            // Find closest bounding box for each tracked people
            for (int i=0; i < (input.poses).size(); i++)
            {
                float x = ((input.poses[i]).position.x);
                float y = ((input.poses[i]).position.y);

                if (verbose) {ROS_INFO("Detected person %i: (x,y) = (%f %f)", i, x, y);}
                int j_min = -1;
                float d_min = limit_dist + 1;
                for (int j=0; j < tracked_people.size(); j++)
                {
                    // ROS_INFO("Tracker (x,y) = (%f %f)", (tracked_people[j]).info[0], (tracked_people[j]).info[1]);
                    // float d = std::sqrt(std::pow(x - (tracked_people[j]).info[0],2)+std::pow(y - (tracked_people[j]).info[1],2));

                    x_pred = (((tracked_people[j]).filter_person).X)[0];
                    y_pred = (((tracked_people[j]).filter_person).X)[1];
                    if (verbose) {ROS_INFO("Tracker (x,y) = (%f %f)", x_pred, y_pred);}
                    float d = std::sqrt( std::pow(x - x_pred,2)+std::pow(y - y_pred,2) );

                    if (verbose) {ROS_INFO("Detected person %i and Box %i | Distance %f", i, j, d);}
                    if ( (d<limit_dist) && (d<d_min))
                    {
                        j_min = j;
                        d_min = d;
                        if (verbose) {ROS_INFO("Closest box is now box %i", j);}
                    }

                }

                // j_min now contains the index of the closest tracked_person from the box
                // need to check if that tracked_person has already been taken by another box
                for (int k=0; k < i; k++)
                {
                    if (index_match[k] == j_min)
                    {
                        j_min = -2;
                        // Already used so we make a copy of the tracked_person and we append it at the end of the list tracked_people
                        /*j_min = tracked_people.size();
                        tracked_people.push_back(tracked_people[index_match[k]]);
                        ((tracked_people[j_min]).filter_person).predict();
                        only_trackers.push_back(input.poses[i]);*/
                        break;
                    }
                }
                if (j_min > -1)
                {
                    if (verbose) {ROS_INFO("Person %i assigned to box %i", i, j_min);}
                    index_match[i] = j_min;
                }
                else if (j_min == -2)
                {
                    if (verbose) {ROS_INFO("Person %i's assigned tracker is already used, discarding this person.", i);}
                    discarded_poses.push_back(i);
                }
                else // A new person appeared in the field of view of the camera
                {
                    if (verbose) {ROS_INFO("A new person has appeared in the field of view!");}
                    tracked_person new_person(x,y);
                    index_match[i] = tracked_people.size();
                    tracked_people.push_back(new_person);
                    ((tracked_people[index_match[i]]).filter_person).predict();
                    only_trackers.push_back(input.poses[i]);
                }
            }


            // FEED MEASUREMENTS TO FILTERS //

            for (int i=0; i < (input.poses).size(); i++)
            {
		if (is_not_discarded(i,discarded_poses)){
                if (verbose) {ROS_INFO("Updating person %i's position thanks to their assigned bounding box.", i);}

                // Reset filter age
                (tracked_people[index_match[i]]).age -= 2;
                if ((tracked_people[index_match[i]]).age < 0)
                {
                    (tracked_people[index_match[i]]).age = 0;
                }

                // Measurement Step
                x_box = ((input.poses[i]).position.x);
                y_box = ((input.poses[i]).position.y);
                vx_box = (x_box - (tracked_people[index_match[i]]).x_prev)/h;
                vy_box = (y_box - (tracked_people[index_match[i]]).y_prev)/h;
                Z << x_box, y_box, vx_box, vy_box;
                (tracked_people[index_match[i]]).x_prev = x_box;
                (tracked_people[index_match[i]]).y_prev = y_box;

                // Prediction Step
                ((tracked_people[index_match[i]]).filter_person).predict();

                // Correction Step
                ((tracked_people[index_match[i]]).filter_person).correct( Z );
                 (tracked_people[index_match[i]]).has_been_updated = true;
                x_pred = (((tracked_people[index_match[i]]).filter_person).X)[0];
                y_pred = (((tracked_people[index_match[i]]).filter_person).X)[1];

                // Update output
                (only_trackers[index_match[i]]).position.x = (x_pred);
                (only_trackers[index_match[i]]).position.y = (y_pred);
            }}

        }
        else
        {
            if (verbose) {ROS_INFO("No one has been detected.");}
        }

        // TRACKING WITH POINT CLOUD // 
	for (int i=0; i < tracked_people.size(); i++)
        {
        if ((tracked_people[i]).has_been_updated == false)
        {
            if (verbose) {ROS_INFO("Person %i has not been detected by the camera during this step.",i);}
            // ((tracked_people[i]).filter_person).correct(); // no measurement
            // (tracked_people[i]).has_been_updated = true;

         // If not in the field of view of the camera then tracking with point cloud
         float x = (((tracked_people[i]).filter_person).X)[0];
         float y = (((tracked_people[i]).filter_person).X)[1];
         std::vector<float> tracked = track_in_pcl(x, y);
         if (tracked[2]==1.0) 
         {
               /*tracked[0] = x;
               tracked[1] = y;  
               std::cout << "Person " << i << " associated with no point, position not modified" << std::endl;*/
               tracked_people.erase(tracked_people.begin() + i); // remove the person from the list
               only_trackers.erase(only_trackers.begin() + i);
               std::cout << "Person " << i << " associated with no point, discarded" << std::endl;
               i--;
               continue;
         }
         else
         {
         std::cout << "Person " << i << " doing mean in pcl at pos (" << x << "," << y << ")" << std::endl;
         std::cout << "Person " << i << " found in pcl at position: (" << tracked[0] << "," << tracked[1] << ")" << std::endl;
         }
         float time_diff = 10*static_cast<float>(ros::Time::now().toSec() - time_start_clock);
         time_start_clock = ros::Time::now().toSec();
         Z << tracked[0], tracked[1], (tracked[0]-x)/time_diff, (tracked[1]-y)/time_diff;
         ((tracked_people[i]).filter_person).correct( Z );
         (tracked_people[i]).has_been_updated = true;

         // Update output (TODO check if it's ok to use i instead of something like index_match[i])
         (only_trackers[i]).position.x = (((tracked_people[i]).filter_person).X)[0];
         (only_trackers[i]).position.y = (((tracked_people[i]).filter_person).X)[1];

        }
        }

        // DISPLAY TRACKED PEOPLE (for information purpose) //

        if (verbose) {ROS_INFO("Number of current trackers: %i", static_cast<int>(only_trackers.size()));}
        for (int iter=0; iter < tracked_people.size(); iter++)
        {
            ((tracked_people[iter]).filter_person).predict();
            if (verbose) {ROS_INFO("Person %i predicted at position: %f %f", iter, (((tracked_people[iter]).filter_person).X)[0], (((tracked_people[iter]).filter_person).X)[1]);}
        }
          
        // PUBLISH OUTPUT (with new information) in velodyne frame //
        if (verbose) {ROS_INFO("There is a total of %i trackers", only_trackers.size());}
        output.poses = only_trackers;
        pub_.publish(output);

        // PUBLISH OUTPUT (with new information) in map frame //
        
        // Check if the pose can be transformed from the frame of the camera to the frame of the map
        ros::Time now = ros::Time::now();
        bool can_transform_to_map = false;
        try
        {
            //listener_.canTransform("map", "my_velodyne_link", ros::Time(0));
            listener_.waitForTransform("map", "velodyne_link", now, ros::Duration(3.0));
            listener_.lookupTransform("map", "velodyne_link", now, transform_);
            can_transform_to_map = true;
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
        }

        if (can_transform_to_map)
        {
            geometry_msgs::PoseArray pose_people_map_filtered; // Remove previous poses from this camera
     
            for (int i=0; i<(output.poses).size(); i++) // For each person detected by this camera this camera
            {
                geometry_msgs::PoseStamped pose_person; // I have to use a PoseStamped because tf cannot work with PoseArray
                pose_person.header = output.header;
                pose_person.header.frame_id = "velodyne_link";
                pose_person.header.stamp = now;
                pose_person.pose = output.poses[i];
                pose_person.pose.orientation.x = 0.0f;
		pose_person.pose.orientation.y = 0.0f;
		pose_person.pose.orientation.z = 0.0f;
		pose_person.pose.orientation.w = 1.0f;

                geometry_msgs::PoseStamped pose_person_map_filtered; // PoseStamped that will received the transformed pose_person
                pose_person_map_filtered.header = output.header;
                pose_person_map_filtered.header.frame_id = "map";
                pose_person_map_filtered.header.stamp = now;
                listener_.transformPose("map", pose_person, pose_person_map_filtered); // Transform from the frame of the camera to the one of the map

                pose_people_map_filtered.poses.push_back(pose_person_map_filtered.pose); // Append to array of poses
            }

            pose_people_map_filtered.header = output.header;
            pose_people_map_filtered.header.frame_id = "map";
            pose_people_map_filtered.header.stamp = now;
            pub_in_map_.publish(pose_people_map_filtered); // Publish poses in map frame
        }

    }

void callback_pcl(const sensor_msgs::PointCloud2& input) // Callback to get 3D point cloud
{
     std::cout << " == Point cloud received == " << std::endl;

     std::vector<std::vector<float>> matrix(input.height*input.width, std::vector<float>(2));

     for (int i=0; i<input.height; i++)
     {
          for (int j=0; j<input.width; j++)
          {
               // Position of the information (X,Y,Z) in the world frame
               // See the documentation of sensor_msgs/PointCloud2 to understand the offsets and steps
               int arrayPosition = i * input.row_step + j * input.point_step;
               // Array Position is the start of a 32 bytes long sequence that contains X, Y, Z and RGB data
               // about the pixel at position (x,y) in the picture
               int arrayPosX = arrayPosition + input.fields[0].offset;
               int arrayPosY = arrayPosition + input.fields[1].offset;
               //int arrayPosZ = arrayPosition + input.fields[2].offset; // Z axis not required
               memcpy(&matrix[i*input.width+j][0], &input.data[arrayPosX], sizeof(float));
               memcpy(&matrix[i*input.width+j][1], &input.data[arrayPosY], sizeof(float));
               //memcpy(&matrix[i*input.width+j][2], &input.data[arrayPosZ], sizeof(float)); // Z axis not required
          }
     }

     point_cloud = matrix;

     /*std::cout << point_cloud.size() << std::endl;

     std::vector<float> tracker = track_in_pcl(0.0f, 2.0f);

     for (int i=0; i<tracker.size(); i++)
     { 
     	std::cout << tracker[i] << "  ";
     }
     std::cout << std::endl;*/

    
    /*for (int i=0; i<tracked_people.size(); i++)
    {
         float x = (((tracked_people[i]).filter_person).X)[0];
         float y = (((tracked_people[i]).filter_person).X)[1];
         std::vector<float> tracked = track_in_pcl(x, y);
         std::cout << "Person " << i << " doing mean in pcl at pos (" << x << "," << y << ")" << std::endl;
         std::cout << "Person " << i << " found in pcl at position: (" << tracked[0] << "," << tracked[1] << ")" << std::endl;
         float time_diff = 10*static_cast<float>(ros::Time::now().toSec() - time_start_clock);
         time_start_clock = ros::Time::now().toSec();
         Z << tracked[0], tracked[1], (tracked[0]-x)/time_diff, (tracked[1]-y)/time_diff;
         ((tracked_people[i]).filter_person).correct( Z );
    }
    for (int i=0; i<tracked_people.size(); i++)
    {
         float x = (((tracked_people[i]).filter_person).X)[0];
         float y = (((tracked_people[i]).filter_person).X)[1];
         float vx = (((tracked_people[i]).filter_person).X)[2];
         float vy = (((tracked_people[i]).filter_person).X)[3];
         std::cout << "Person " << i << " tracked with pcl (after Kalman correction): (" << x << "," << y << ")" << std::endl;
         std::cout << "with velocity: (" << vx << "," << vy << ")" << std::endl;
         // Prediction Step
         ((tracked_people[i]).filter_person).predict();
    }*/
}

std::vector<float> track_in_pcl(float const& x, float const& y)
{
     const float radius_pow = std::pow(0.5,2); // square of radius of cylinder in meter     
     std::vector<float> output(3,0.0);
     int point_count = 0;

     for (int i=0; i<point_cloud.size(); i++)
     {
           if ((std::pow(point_cloud[i][0]-x,2)+std::pow(point_cloud[i][1]-y,2))<=radius_pow)
           {
                output[0] += point_cloud[i][0];
                output[1] += point_cloud[i][1];  
                point_count++;         
           }
     }
     if (point_count!=0) {
     output[0] /= point_count;
     output[1] /= point_count; }
     else {output[2] = 1.0;} // flag to know that there is 0 point}

     std::cout << point_count << " points have been considered as belonging to the person." << std::endl;

     return output;
}

bool is_not_discarded(int const& i, std::vector<int> discarded_poses)
{
    for (int k=0; k<discarded_poses.size(); k++)
    {
         if (i == discarded_poses[k]) {return false;}
    }
    return true;
}
private:
    ros::NodeHandle n_;   // Node handle
    ros::Publisher pub_;  // To publish the filtered people's position
    ros::Publisher pub_in_map_;  // To publish the filtered people's position
    ros::Subscriber sub_; // To listen to people's position detected by Realsense cameras

    KalmanFilter filter1;
    std::vector<tracked_person> tracked_people;
    std::vector<geometry_msgs::Pose> only_trackers;
    std::vector<int> discarded_poses;
    std::vector<std::vector<float>> point_cloud;
    ros::Subscriber sub_pcl;

    tf::TransformListener listener_; // To listen to transforms
    tf::StampedTransform transform_; // Transform object

    double time_start_clock;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "kalman_2d_node_simplified");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    ros::spin();

    return 0;
}



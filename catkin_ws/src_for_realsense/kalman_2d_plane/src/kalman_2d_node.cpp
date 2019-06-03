#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "KalmanFilter.h"

// Eigen library and the header that contains my functions
#include <eigen3/Eigen/Core>

#include <fstream>  // To write data into files
#include <limits>

typedef Eigen::Matrix<int8_t, 1, Eigen::Dynamic> MatrixXi8_layer;
typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> myMap;

using namespace std;

const int limit_age = 25;
const int born_age = limit_age - 4;
const int limit_dist = 1;
const bool verbose = false;

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
    sigma_1 = 0.25;
    sigma_2 = 0.25;
    sigma_vx = 0.3;
    sigma_vy = 0.3;
    sigma_0 = 0.5;
    h = 0.125f;

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
        pub_ = n_.advertise<geometry_msgs::PoseArray>("/pose_people_map_filtered", 5);

        // Subscribing to people's position detected by Realsense cameras
        sub_ = n_.subscribe("/pose_people_map", 1, &SubscribeAndPublish::callback, this);

        /* Set the parameters (standard deviations) */
        Z = Eigen::VectorXf(n_states);
        KalmanFilter filter_temp(n_states,0);
        filter1 = filter_temp;

        /* Initialize the Filter*/
        filter1.setFixed(A, H, Q, R);
        filter1.setInitial(X0, P0);

        ROS_INFO("Kalman filter for 2D horizontal plane has been initialized.");
    }

    void callback(const geometry_msgs::PoseArray& input)
    {

        if (verbose) {ROS_INFO("Receiving raw bounding boxes");}

        geometry_msgs::PoseArray output = input;

        // REMOVE OLD TRACKERS //

        for (int i=0; i < tracked_people.size(); i++)
        {
            (tracked_people[i]).has_been_updated = false;
            (tracked_people[i]).age += 1;
            if (verbose) {ROS_INFO("Filter %i is now %i steps old", i, (tracked_people[i]).age);}
            if (((tracked_people[i]).age) > limit_age)
            {
                tracked_people.erase(tracked_people.begin() + i); // remove the person from the list
                only_trackers.erase(only_trackers.begin() + i);
                if (verbose) {ROS_INFO("Filter %i deleted because it is too old", i);}
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
                        // Already used so we make a copy of the tracked_person and we append it at the end of the list tracked_people
                        j_min = tracked_people.size();
                        tracked_people.push_back(tracked_people[index_match[k]]);
                        ((tracked_people[j_min]).filter_person).predict();
                        only_trackers.push_back(input.poses[i]);
                        break;
                    }
                }
                if (j_min != -1)
                {
                    if (verbose) {ROS_INFO("Person %i assigned to box %i", i, j_min);}
                    index_match[i] = j_min;
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
            }


	for (int i=0; i < tracked_people.size(); i++)
    {
        if ((tracked_people[i]).has_been_updated == false)
        {
            ((tracked_people[i]).filter_person).correct(); // no measurement
            (tracked_people[i]).has_been_updated = true;
        }
    }

        }
        else
        {
            if (verbose) {ROS_INFO("No one has been detected.");}
        }

        // DISPLAY TRACKED PEOPLE (for information purpose) //

        if (verbose) {ROS_INFO("Number of current trackers: %i", static_cast<int>(only_trackers.size()));}
        for (int iter=0; iter < tracked_people.size(); iter++)
        {
            ((tracked_people[iter]).filter_person).predict();
            if (verbose) {ROS_INFO("Person %i predicted at position: %f %f", iter, (((tracked_people[iter]).filter_person).X)[0], (((tracked_people[iter]).filter_person).X)[1]);}
        }

        // PUBLISH OUTPUT (with new information) //

        output.poses = only_trackers;
        pub_.publish(output);
    }

private:
    ros::NodeHandle n_;   // Node handle
    ros::Publisher pub_;  // To publish the filtered people's position
    ros::Subscriber sub_; // To listen to people's position detected by Realsense cameras

    KalmanFilter filter1;
    std::vector<tracked_person> tracked_people;
    std::vector<geometry_msgs::Pose> only_trackers;


};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "kalman_2d_node");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    ros::spin();

    return 0;
}



#include "ros/ros.h"
#include "object_msgs/ObjectsInBoxes.h"
#include "KalmanFilter.h"

// Eigen library and the header that contains my functions
#include <eigen3/Eigen/Core>

#include <fstream>  // To write data into files

typedef Eigen::Matrix<int8_t, 1, Eigen::Dynamic> MatrixXi8_layer;
typedef Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> myMap;

using namespace std;

const int limit_age = 25;
const int limit_dist = 250;

/* Initialization */
int n_states = 4;
float x_box, y_box, x_prev, y_prev, x_pred, y_pred;
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
	sigma_1 = 5;
	sigma_2 = 5;
	sigma_vx =  25;
	sigma_vy =  25;
	sigma_0 = 50;
	h = 0.1;

        /* Set Matrix and Vector for Kalman Filter: */
        
        A << 1, 0, h, 0,
        0, 1, 0, h,
        0, 0, 1, 0,
        0, 0, 0, 1;

        
        H << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 1;

        
        Q << h*sigma_vx,          0,        0,        0,
             0, h*sigma_vy,        0,        0,
             0,          0, sigma_vx,        0,
             0,          0,        0, sigma_vy;

        
        R << sigma_1,       0,         0,         0,
             0, sigma_2,         0,         0,
             0,       0, sigma_1/h,         0,
             0,       0,         0, sigma_2/h;
        
        X0 << 100,150,0,0;
        
        P0 << 0, 0,       0,       0,
              0, 0,       0,       0,
              0, 0, sigma_0,       0,
              0, 0,       0, sigma_0;

}

// DECLARATION STUCTURE TRACKED_PERSON
struct tracked_person
{
   int age; // number of steps since this person was updated
   float info[4]; // [x, y, height, width] vector of the bounding box
   KalmanFilter filter_person;

   tracked_person(float x, float y, float h, float w);  
};

tracked_person::tracked_person(float x, float y, float h, float w)
{
     info[0] = x; info[1] = y; info[2] = h, info[3] = w;
     age = limit_age - 6;
     KalmanFilter filter_temp(n_states,0);
     filter_person = filter_temp;
     /* Initialize the Filter*/
     filter_person.setFixed(A, H, Q, R);
     X0 << x,y,0,0;
     filter_person.setInitial(X0, P0);
}
// END STUCTURE TRACKED_PERSON



class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
	// Initialization of Kalman parameters
	init_parameters_kalman();

        //Topic you want to publish
        pub_ = n_.advertise<object_msgs::ObjectsInBoxes>("/filtered_boxes", 10);

        //Topic you want to subscribe
        sub_ = n_.subscribe("/movidius_ncs_nodelet/detected_objects", 1, &SubscribeAndPublish::callback, this);

        /* Set the parameters (standard deviations) */
        Z = Eigen::VectorXf(n_states);
        KalmanFilter filter_temp(n_states,0);
        filter1 = filter_temp;

        /* Initialize the Filter*/
        filter1.setFixed(A, H, Q, R);
        filter1.setInitial(X0, P0);


    }

    // TODO:
    // Use list of filter
    // A filter is created when a new person is detected and deleted when they are not used anymore
    // Bounding boxes are assigned to the nearest existing filter

    void callback(const object_msgs::ObjectsInBoxes& input)
    {

        ROS_INFO("Receiving raw bounding boxes");

        object_msgs::ObjectsInBoxes output = input;        

        // KEEP ONLY PEOPLE AND DISCARD OTHER OBJECTS

        std::vector<object_msgs::ObjectInBox> only_people;
        

        for (int iter=0; iter < (input.objects_vector).size(); iter++)
        {
            if ( (input.objects_vector[iter]).object.object_name == "person" )
            {
		only_people.push_back(input.objects_vector[iter]);
                ROS_INFO("Detected person at position: %f %f", static_cast<float>((input.objects_vector[iter]).roi.x_offset), static_cast<float>((input.objects_vector[iter]).roi.y_offset));
            }
        }

        // REMOVE OLD BOUNDING BOXES

        for (int i=0; i < tracked_people.size(); i++)
        {
            (tracked_people[i]).age += 1;
	    ROS_INFO("Filter %i is now %i steps old", i, (tracked_people[i]).age);
            if (((tracked_people[i]).age) > limit_age) 
		{
		     tracked_people.erase(tracked_people.begin() + i); // remove the person from the list  
                     only_trackers.erase(only_trackers.begin() + i);               
		     ROS_INFO("Filter %i deleted because it is too old", i);
		     i -= 1; // go on step back since an element has been removed
		}
        }

        // PAIR DETECTED PEOPLE WITH THE LIST OF TRACKED PEOPLE
	// For each detected people
	// - find closest tracked people
        // - if two detected people share the same tracked people, a copy is created for the second one
        // - if a tracked people has no associated tracked people (there are all out of the limit range), a new tracked_people is created
        if (only_people.size() != 0)
        {
        std::vector<int> index_match(only_people.size(), -1);
        // Find closest bounding box for each tracked people
        for (int i=0; i < only_people.size(); i++) 
        {
	     float x = static_cast<float>((only_people[i]).roi.x_offset);
             float y = static_cast<float>((only_people[i]).roi.y_offset);
	     int j_min = -1;
             float d_min = limit_dist + 1;
             for (int j=0; j < tracked_people.size(); j++) 
             {
		float d = std::sqrt(std::pow(x - (tracked_people[j]).info[0],2)+std::pow(y - (tracked_people[j]).info[1],2));
                if ( (d<limit_dist) && (d<d_min))
		{
			j_min = j;
			d_min = d;
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
                             only_trackers.push_back(only_people[i]);
			     break;
             		}
		}
                if (j_min != -1)
                {
                	index_match[i] = j_min;
		}
		else // A new person appeared in the field of view of the camera
		{
			tracked_person new_person(x,y,static_cast<float>((only_people[i]).roi.height), static_cast<float>((only_people[i]).roi.width));
                        index_match[i] = tracked_people.size();
                        tracked_people.push_back(new_person);
                        only_trackers.push_back(only_people[i]);
		}
        }
	

	// FEED MEASUREMENTS TO FILTERS

	for (int i=0; i < only_people.size(); i++) 
        {
	    
	    // Reset filter age
	    (tracked_people[index_match[i]]).age -= 2;
            if ((tracked_people[index_match[i]]).age < 0)
	    {
		(tracked_people[index_match[i]]).age = 0;
	    }

	    // Measurement Step
	    x_box = static_cast<float>((only_people[i]).roi.x_offset);
            y_box = static_cast<float>((only_people[i]).roi.y_offset);
            vx_box = (x_box - x_prev)/h;
            vy_box = (y_box - y_prev)/h;
	    Z << x_box, y_box, vx_box, vy_box; 
            x_prev = x_box;
            y_prev = y_box;
	
	    // Prediction Step
            ((tracked_people[index_match[i]]).filter_person).predict();

	    // Correction Step
            ((tracked_people[index_match[i]]).filter_person).correct( Z ); 
            x_pred = (((tracked_people[index_match[i]]).filter_person).X)[0];
            y_pred = (((tracked_people[index_match[i]]).filter_person).X)[1];

	    // Update output
            (only_trackers[index_match[i]]).roi.x_offset = static_cast<int>(std::round(x_pred));
            (only_trackers[index_match[i]]).roi.y_offset = static_cast<int>(std::round(y_pred));
            (only_trackers[index_match[i]]).roi.height = (only_people[i]).roi.height;
            (only_trackers[index_match[i]]).roi.width  = (only_people[i]).roi.width;
	}

	
        }
        else
        {
            ROS_INFO("No one has been detected.");
	}

	// DISPLAY TRACKED PEOPLE (for information purpose)
        ROS_INFO("Number of current trackers: %i", static_cast<int>(only_trackers.size()));
	for (int iter=0; iter < tracked_people.size(); iter++)
        {
            ((tracked_people[iter]).filter_person).predict();
            ROS_INFO("Person %i predicted at position: %f %f", iter, (((tracked_people[iter]).filter_person).X)[0], (((tracked_people[iter]).filter_person).X)[1]);
        }
        
      
        /*if (only_people.size() == 0)
        {
            ROS_INFO("No one has been detected.");
            // Output predictions

		for (int i_track=0; i_track < tracked_people.size(); i_track ++)
		{
			(tracked_people[i_track]).predict()
		}
            filter1.predict(); // Prediction step
            ROS_INFO("Predicted box at position: %f %f", (filter1.X)[0], (filter1.X)[1]);
            //TODO
        }
        else
        {
        for (int iter=0; iter < only_people.size(); iter++)
        {
            ROS_INFO("At least one person has been detected.");
            x_box = static_cast<float>((only_people[iter]).roi.x_offset);
            y_box = static_cast<float>((only_people[iter]).roi.y_offset);
            vx_box = (x_box - x_prev)/h;
            vy_box = (y_box - y_prev)/h;
	    Z << x_box, y_box, vx_box, vy_box; // Measurement
            x_prev = x_box;
            y_prev = y_box;
            filter1.predict();    // Prediction step
            ROS_INFO("Predicted box at position: %f %f", (filter1.X)[0], (filter1.X)[1]);
            filter1.correct( Z ); // Correction step
            x_pred = (filter1.X)[0];
            y_pred = (filter1.X)[1];
            ROS_INFO("Corrected box at position: %f %f", x_pred, y_pred);
            
            (only_people[iter]).roi.x_offset = static_cast<int>(std::round(x_pred));
            (only_people[iter]).roi.y_offset = static_cast<int>(std::round(y_pred));
        }
        }*/

        // Publish output with new information
        output.objects_vector = only_trackers;
        pub_.publish(output);
    }

private:
    ros::NodeHandle n_;   // Node handle
    ros::Publisher pub_;  // To publish filtered boxes
    ros::Subscriber sub_; // To listen to raw boxes

    KalmanFilter filter1;
    std::vector<tracked_person> tracked_people;
    std::vector<object_msgs::ObjectInBox> only_trackers;


};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "kalman_bounding_boxes_node");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    ros::spin();

    return 0;
}



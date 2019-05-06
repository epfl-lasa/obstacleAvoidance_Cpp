/*
 * Copyright (C) 2017 chapulina
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/common/Plugin.hh>

// Adding some stuff to make the link with ROS
#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseArray.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Events.hh>
///

namespace gazebo
{
  class GAZEBO_VISIBLE SetStaticPose : public ModelPlugin
  {
    
    /// \brief Constructor
    public:  SetStaticPose();

    /// \brief Destructor
    public: ~SetStaticPose();

    // Documentation inherited
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Main loop to update the pose
    private: void OnUpdate();

    /// \brief All the event connections.
    private: std::vector<event::ConnectionPtr> connections;

    /// \brief Pointer to the model
    private: physics::ModelPtr model;

    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A publisher to a named topic.
    //private: transport::PublisherPtr pub;

   
    private: // Parameters
	float id_person = 0;
        float pos_init_x = 0;
        float pos_init_y = 0;
        float pos_final_x = 0;
        float pos_final_y = 0;
        double time_back_forth = 0;
        double initial_time = 0;

    /// \brief topic name
    private: std::string topic_name;

    /// \brief frame name
    private: std::string frame_name;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;
    private: ros::Subscriber sub_;

    private: void Callback(const geometry_msgs::PoseArrayConstPtr &msg);
    private: ros::CallbackQueue subscriber_queue_;
    private: void SubscriberQueueThread();
    private: boost::thread subscriber_queue_thread_;
    
    /// \brief ros message
    private: geometry_msgs::Vector3Stamped range_msg_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    private: ros::CallbackQueue range_queue_;
    private: void RangeQueueThread();
    private: boost::thread callback_queue_thread_;

    /// \brief deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    public: void LoadThread();
    private: boost::thread deferred_load_thread_;
    private: unsigned int seed;

    /// \brief Placeholder functions
    private: void RangeConnect();
    private: void RangeDisconnect();




  };
}

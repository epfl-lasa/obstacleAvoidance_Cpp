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

#include <ignition/math/Pose3.hh>
#include <ignition/math/Rand.hh>
#include <gazebo/physics/physics.hh>
#include "SetStaticPose.hh"

#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <cmath>

#include "gazebo_plugins/gazebo_ros_range.h"
#include "gazebo_plugins/gazebo_ros_utils.h"

#include <sdf/sdf.hh>
#include <sdf/Param.hh>

#include <tf/tf.h>


using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SetStaticPose)

////////////////////////////////////////////////////////////////////////////////
// Constructor
SetStaticPose::SetStaticPose()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
SetStaticPose::~SetStaticPose()
{
  this->range_queue_.clear();
  this->range_queue_.disable();
  this->subscriber_queue_.clear();
  this->subscriber_queue_.disable();

  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
  this->subscriber_queue_thread_.join();

  delete this->rosnode_;
}

/////////////////////////////////////////////////
void SetStaticPose::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->model = _model;

  this->connections.push_back(event::Events::ConnectWorldUpdateEnd(
      std::bind(&SetStaticPose::OnUpdate, this)));
  
  if (!_sdf->HasElement("id_person")) {
    std::cout << "id_person has been set to default value (not good except if only 1 person)." << std::endl;
    id_person = 0.0;
  } else {
    id_person = _sdf->GetElement("id_person")->Get<float>();
  }

  if (!_sdf->HasElement("pos_init_x")) {
    std::cout << "Initial pos_x has been set to default value." << std::endl;
    pos_init_x = 1.0;
  } else {
    pos_init_x = _sdf->GetElement("pos_init_x")->Get<float>();
  }

    if (!_sdf->HasElement("pos_init_y")) {
    std::cout << "Initial pos_y has been set to default value." << std::endl;
    pos_init_y = 1.0;
  } else {
    pos_init_y = _sdf->GetElement("pos_init_y")->Get<float>();
  }

    if (!_sdf->HasElement("pos_final_x")) {
    std::cout << "Final pos_x has been set to default value." << std::endl;
    pos_final_x = 2.0;
  } else {
    pos_final_x = _sdf->GetElement("pos_final_x")->Get<float>();
  }

    if (!_sdf->HasElement("pos_final_y")) {
    std::cout << "Final pos_y has been set to default value." << std::endl;
    pos_final_y = 3.0;
  } else {
    pos_final_y = _sdf->GetElement("pos_final_y")->Get<float>();
  }


    if (!_sdf->HasElement("time_back_forth")) {
    std::cout << "Time for one back-and-forth has been set to default value." << std::endl;
    time_back_forth = 3.0;
  } else {
    time_back_forth = _sdf->GetElement("time_back_forth")->Get<double>();
  }

   common::Time tempo = common::Time::GetWallTime();
   initial_time = tempo.Double();

   // Create the node
   this->node = transport::NodePtr(new transport::Node());

   this->node->Init();//this->model->GetWorld()->GetName());
     
   // Create a topic name
   this->topic_name = "/gazebo/pos_moving_people";

   // Create a frame name
   this->frame_name = "base_link";
 
   // Create publisher
   //this->pub_ = node->Advertise<gazebo::msgs::Pose>(this->topic_name);//transport::PublisherPtr(new transport::Publisher(topicName, "PoseStamped", 1, 10));

   std::cout << "Model: " << this->model->GetName() << " with " << id_person << " " << pos_init_x << " " << pos_init_y << " " << pos_final_x << " " << pos_final_y << std::endl;


   // Init ROS
   if (ros::isInitialized())
   {
    // ros callback queue for processing subscription
    this->deferred_load_thread_ = boost::thread(
      boost::bind(&SetStaticPose::LoadThread, this));
   }
    else
   {
    gzerr << "Not loading plugin since ROS hasn't been "
          << "properly initialized.  Try starting gazebo with ros plugin:\n"
          << "  gazebo -s libgazebo_ros_api_plugin.so\n";
   }
}

////////////////////////////////////////////////////////////////////////////////
// Subscriber callback
void SetStaticPose::Callback(const geometry_msgs::PoseArrayConstPtr &msg) 
{
  /// TODO
  std::cout << "Callback of PoseArray has been triggered" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void SetStaticPose::LoadThread()
{
  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->frame_name = "map";//tf::resolve(prefix, this->frame_name);

  if (this->topic_name != "")
  {
    ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<geometry_msgs::Vector3Stamped>(
      this->topic_name, 1,
      boost::bind(&SetStaticPose::RangeConnect, this),
      boost::bind(&SetStaticPose::RangeDisconnect, this),
      ros::VoidPtr(), &this->range_queue_);
    this->pub_ = this->rosnode_->advertise(ao);
    std::cout << "Publisher of SetStaticPose has been initialized" << std::endl;
  }

  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::PoseArray>(
      "/poses_for_gazebo", 1,
      boost::bind(&SetStaticPose::Callback, this, _1),
      ros::VoidPtr(), &this->subscriber_queue_);

  this->sub_ = this->rosnode_->subscribe(so);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void SetStaticPose::RangeConnect()
{
   // Placeholder
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void SetStaticPose::RangeDisconnect()
{
   // Placeholder
}



/////////////////////////////////////////////
void SetStaticPose::OnUpdate()
{
  // Get the desired pose, here giving a random offset
  ignition::math::Pose3d pose = this->model->GetWorldPose().Ign();

  /*pose += ignition::math::Pose3d(ignition::math::Rand::DblUniform(-0.01, 0.01),
                                 ignition::math::Rand::DblUniform(-0.01, 0.01),
                                 ignition::math::Rand::DblUniform(-0.01, 0.01),
                                 ignition::math::Rand::DblUniform(-0.01, 0.01),
                                 ignition::math::Rand::DblUniform(-0.01, 0.01),
                                 ignition::math::Rand::DblUniform(-0.01, 0.01));*/
    //pose += ignition::math::Pose3d(0, 0, 0, 0, 0, 0.003);

  // Don't let it go far under the gound
  // pose.Pos().Z() = pose.Pos().Z() < 0.5 ? 0.5 : pose.Pos().Z();

  // Small back-and-forth test
  common::Time tempo = common::Time::GetWallTime();
  // std::cout << tempo.Float() << std::endl;
  double remain = std::remainder( tempo.Double() - initial_time, time_back_forth);
  // std::cout << remain << std::endl;
  float next_x = pos_init_x + (pos_final_x - pos_init_x) * (std::abs(static_cast<float>(remain))/time_back_forth) * 2;
  float next_y = pos_init_y + (pos_final_y - pos_init_y) * (std::abs(static_cast<float>(remain))/time_back_forth) * 2;
  pose.Pos().X() = next_x;
  pose.Pos().Y() = next_y;

  // Publish position
  /*ignition::math::Pose3d vec = ignition::math::Pose3d( static_cast<double>(next_x), static_cast<double>(next_y), 0.0, 0.0, 0.0, 0.0);
  gazebo::msgs::Pose vec_msg = gazebo::msgs::Convert(vec);
  //gazebo::msgs::Stamp(gazebo::msgs::GetHeader(vec_msg));
  this->pub->Publish(vec_msg); std::cout << "Publishing" << std::endl;*/

  // Add Frame Name
  this->range_msg_.header.frame_id = "map";
  this->range_msg_.header.stamp.sec = tempo.sec;
  this->range_msg_.header.stamp.nsec = tempo.nsec;

  // Add data to ros message
  this->range_msg_.vector.x = next_x;
  this->range_msg_.vector.y = next_y;
  this->range_msg_.vector.z = this->id_person;

  // send data out via ros message
  if (this->topic_name != "")
  { 
      //this->pub_.publish(this->range_msg_); 
      //std::cout << "Will publish on topic " << this->pub_.getTopic() << std::endl;
      //std::cout << "Num of subscribers " << this->pub_.getNumSubscribers() << std::endl;
      if (this->pub_.getNumSubscribers() > 0) {this->pub_.publish(this->range_msg_);}
  }

  this->model->SetWorldPose(pose);
}

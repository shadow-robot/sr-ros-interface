/**
 * @file   srh_example_controller.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Mon Nov  7 10:35:04 2011
 *
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*
 * @brief This is a simple controller example, showing you how to create your
 *        own controller, how to receive the data and update the command.
 *
 */

#include "../example/srh_example_controller.hpp"
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

#include <std_msgs/Float64.h>

//Register the controller to be able to load it with the controller manager.
PLUGINLIB_EXPORT_CLASS( controller::SrhExampleController, controller_interface::ControllerBase)

using namespace std;

namespace controller {

  SrhExampleController::SrhExampleController()
    : SrController()
  {
    //This constructor is kept empty: the init functions
    // are called by the controller manager when the controllers are started.
  }

  SrhExampleController::~SrhExampleController()
  {
    //Stop subscribing to the command topic.
    sub_command_.shutdown();
  }

  bool SrhExampleController::init(ros_ethercat_model::RobotState *robot, ros::NodeHandle &n)
  {
    assert(robot);
    node_ = n;

    //read the joint we're controlling from the parameter server.
    std::string joint_name;
    if (!node_.getParam("joint", joint_name)) {
      ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }

    //set the state publisher for this controller:
    // This publisher publishes interesting data for the current controller.
    // (useful for debugging / tuning)
    // Feel free to create a different message type, to publish more meaningful
    // information for your controller (cf srh_mixed_position_velocity_controller.cpp)
    controller_state_publisher_.reset(
      new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
      (node_, "state", 1));

    //Calls the 2nd init function to finish initializing
    return init(robot, joint_name);
  }

  bool SrhExampleController::init(ros_ethercat_model::RobotState *robot, const std::string &joint_name)
  {
    assert(robot);
    robot_ = robot;

    //We need to store 2 different joint states for the joint 0s:
    // They control the distal and the middle joint with the same control.
    if( joint_name.substr(3,1).compare("0") == 0)
    {
      has_j2 = true;

      //The joint 0 is name *FJ0, and is controlling *J1 + *J2.
      std::string j1 = joint_name.substr(0,3) + "1";
      std::string j2 = joint_name.substr(0,3) + "2";

      //Get the pointer to the joint state for *J1
      joint_state_ = robot_->getJointState(j1);
      if (!joint_state_)
      {
        ROS_ERROR("SrhMixedPositionVelocityController could not find joint named \"%s\"\n",
                  joint_name.c_str());
        return false;
      }

      //Get the pointer to the joint state for *J2
      joint_state_2 = robot_->getJointState(j2);
      if (!joint_state_2)
      {
        ROS_ERROR("SrhMixedPositionVelocityController could not find joint named \"%s\"\n",
                  joint_name.c_str());
        return false;
      }
    }
    else //"normal" joints: one controller controls one joint
    {
      has_j2 = false;

      //get the pointer to the joint state
      joint_state_ = robot_->getJointState(joint_name);
      if (!joint_state_)
      {
        ROS_ERROR("SrhMixedPositionVelocityController could not find joint named \"%s\"\n",
                  joint_name.c_str());
        return false;
      }
    }

    // after init creates the subscriber to the /command topic
    after_init();
    return true;
  }


  void SrhExampleController::starting(const ros::Time& time)
  {
    //Here we set the command to be = to the current position
    if( has_j2 ) //if it's *J0, then pos = *J1->pos + *J2->pos
      command_ = joint_state_->position_ + joint_state_2->position_;
    else
      command_ = joint_state_->position_;
  }

  void SrhExampleController::update(const ros::Time& time, const ros::Duration& period)
  {
    assert(robot_ != NULL);
    assert(joint_state_->joint_);

    //make sure the controller has been initialised,
    // to avoid sending a crazy command.
    if (!initialized_)
    {
      starting(time);

      initialized_ = true;
    }

    //compute the commanded effort you want to send
    // to the motor: you can use whatever algorithm
    // you want. To see more complex examples on how
    // to use a pid loop / more than one loop, just
    // go to the src directory, and have a look at
    // srh_mixed_position_velocity_controller.cpp

    //we start by computing the position error
    double error_position = 0.0;
    if( has_j2 )
    {
      //For *J0, the position error is equal to the command - (*J1 + *J2)
      error_position = command_ - (joint_state_->position_ + joint_state_2->position_);
    }
    else
      error_position = command_ - joint_state_->position_;

    //Here I'm simply doing a dummy P controller, with a fixed gain.
    // It can't be used in the real life obviously. That's where you
    // should WRITE YOUR ALGORITHM
    double commanded_effort = 10* error_position;

    //Update the commanded effort.
    if( has_j2 ) //The motor in *J0 is attached to the *J2
      joint_state_2->commanded_effort_ = commanded_effort;
    else
      joint_state_->commanded_effort_ = commanded_effort;

    if(loop_count_ % 10 == 0) //publishes the joint state at 100Hz
    {
      if(controller_state_publisher_ && controller_state_publisher_->trylock())
      {
        controller_state_publisher_->msg_.header.stamp = time;
        controller_state_publisher_->msg_.set_point = command_;

        if( has_j2 )
        {
          controller_state_publisher_->msg_.process_value = joint_state_->position_ + joint_state_2->position_;
          controller_state_publisher_->msg_.process_value_dot = joint_state_->velocity_ + joint_state_2->velocity_;
        }
        else
        {
          controller_state_publisher_->msg_.process_value = joint_state_->position_;
          controller_state_publisher_->msg_.process_value_dot = joint_state_->velocity_;
        }

        controller_state_publisher_->msg_.error = error_position;
        controller_state_publisher_->msg_.time_step = period.toSec();
        controller_state_publisher_->msg_.command = commanded_effort;

        controller_state_publisher_->unlockAndPublish();
      }
    }
    loop_count_++;
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/



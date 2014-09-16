/**
 * @file   srh_syntouch_controllers.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Dec  6 12:01:15 2011
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
 * @brief Dummy controller to show how to use the biotac tactiles to compute the force demand.
 *
 */

#include "../example/srh_syntouch_controllers.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"
#include <sstream>
#include <math.h>
#include <sr_utilities/sr_math_utils.hpp>
#include <std_msgs/Float64.h>

PLUGINLIB_EXPORT_CLASS( controller::SrhSyntouchController, controller_interface::ControllerBase)

using namespace std;

namespace controller {

  SrhSyntouchController::SrhSyntouchController()
    : SrController()
  {
  }

  SrhSyntouchController::~SrhSyntouchController()
  {
    sub_command_.shutdown();
  }

  bool SrhSyntouchController::init(ros_ethercat_model::RobotState *robot, ros::NodeHandle &n)
  {
    ROS_ASSERT(robot);
    robot_ = robot;
    node_ = n;

    if (!node_.getParam("joint", joint_name_)) {
      ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }

    ROS_DEBUG(" --------- ");
    ROS_DEBUG_STREAM("Init: " << joint_name_);

    joint_state_ = robot_->getJointState(joint_name_);
    if (!joint_state_)
    {
      ROS_ERROR("SrhMixedPositionVelocityController could not find joint named \"%s\"\n",
                joint_name_.c_str());
      return false;
    }
    if (!joint_state_->calibrated_)
    {
      ROS_ERROR("Joint %s not calibrated for SrhSyntouchController", joint_name_.c_str());
      return false;
    }

    //init the pointer to the biotacs data, updated at 1kHz
    actuator_ = static_cast<sr_actuator::SrMotorActuator*>( robot->getActuator( joint_name_ ) );

    after_init();
    return true;
  }


  void SrhSyntouchController::starting(const ros::Time& time)
  {
    command_ = joint_state_->position_;

    ROS_WARN_STREAM("Reseting PID for joint  " << joint_state_->joint_->name);
  }

  void SrhSyntouchController::update(const ros::Time& time, const ros::Duration& period)
  {
    if (!joint_state_->calibrated_)
      return;

    ROS_ASSERT(robot_);
    ROS_ASSERT(joint_state_->joint_);

    if (initialized_)
      command_ = joint_state_->commanded_position_;
    else
    {
      initialized_ = true;
      command_ = joint_state_->position_;
    }

    ////////////
    // POSITION
    /////
    //Compute position error:
    double error_position = command_ - joint_state_->position_;

    ////////////
    // TACTILES
    /////
    //you have access here to the whole data coming from the 5 tactiles at full speed.
    double my_first_finger_tactile_pac0 = actuator_->motor_state_.tactiles_->at(0).biotac.pac0;
    if(loop_count_ % 10 == 0)
    {
      ROS_ERROR_STREAM("PAC0, tactile " << my_first_finger_tactile_pac0);
    }

    ////////////
    // EFFORT
    /////
    //Compute the commanded effort to send to the motor
    double commanded_effort = 0.0;
    //TODO: compute the force demand by combining the information you
    // want. You can have a look at the mixed controller to see a
    // working implementation of a controller using different pid loops.

    joint_state_->commanded_effort_ = commanded_effort;

    if(loop_count_ % 10 == 0)
    {
      if(controller_state_publisher_ && controller_state_publisher_->trylock())
      {
        controller_state_publisher_->msg_.header.stamp = time;
        controller_state_publisher_->msg_.set_point = command_;

        controller_state_publisher_->msg_.process_value = joint_state_->position_;
        controller_state_publisher_->msg_.process_value_dot = joint_state_->velocity_;

        controller_state_publisher_->msg_.error = error_position;
        controller_state_publisher_->msg_.time_step = period.toSec();

        controller_state_publisher_->msg_.command = commanded_effort;
        controller_state_publisher_->msg_.measured_effort = joint_state_->effort_;

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



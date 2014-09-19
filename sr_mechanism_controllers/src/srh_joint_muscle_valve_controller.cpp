/**
 * @file   srh_joint_muscle_valve_controller.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Aug 17 12:32:01 2011
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
 * @brief Compute an effort demand from the effort error. As the
 *  effort PID loop is running on the motor boards, there's no PID
 *  loops involved here. We're just using the friction compensation
 *  algorithm to take into account the friction of the tendons.
 *
 */

#include "sr_mechanism_controllers/srh_joint_muscle_valve_controller.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"
#include <sstream>
#include <math.h>
#include "sr_utilities/sr_math_utils.hpp"

#include <std_msgs/Float64.h>

PLUGINLIB_EXPORT_CLASS(controller::SrhJointMuscleValveController, controller_interface::ControllerBase)

using namespace std;

namespace controller
{

SrhJointMuscleValveController::SrhJointMuscleValveController()
  : cmd_valve_muscle_min_(-4),
  cmd_valve_muscle_max_(4)
{
}

bool SrhJointMuscleValveController::init(ros_ethercat_model::RobotState *robot, ros::NodeHandle &n)
{
  ROS_ASSERT(robot);
  robot_ = robot;
  node_ = n;

  if (!node_.getParam("joint", joint_name_))
  {
    ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }

  controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<sr_robot_msgs::JointMuscleValveControllerState>
                                    (node_, "state", 1));

  ROS_DEBUG(" --------- ");
  ROS_DEBUG_STREAM("Init: " << joint_name_);

  // joint 0s e.g. FFJ0
  has_j2 = is_joint_0();
  if (has_j2)
  {
    get_joints_states_1_2();
    if (!joint_state_)
    {
      ROS_ERROR("SrhJointMuscleValveController could not find the first joint relevant to \"%s\"\n", joint_name_.c_str());
      return false;
    }
    if (!joint_state_2)
    {
      ROS_ERROR("SrhJointMuscleValveController could not find the second joint relevant to \"%s\"\n", joint_name_.c_str());
      return false;
    }
  }
  else
  {
    joint_state_ = robot_->getJointState(joint_name_);
    if (!joint_state_)
    {
      ROS_ERROR("SrhJointMuscleValveController could not find joint named \"%s\"\n", joint_name_.c_str());
      return false;
    }
  }

  friction_compensator.reset(new sr_friction_compensation::SrFrictionCompensator(joint_name_));

  //serve_set_gains_ = node_.advertiseService("set_gains", &SrhJointMuscleValveController::setGains, this);
  serve_reset_gains_ = node_.advertiseService("reset_gains", &SrhJointMuscleValveController::resetGains, this);

  ROS_DEBUG_STREAM(" joint_state name: " << joint_state_->joint_->name);
  ROS_DEBUG_STREAM(" In Init: " << getJointName() << " This: " << this
                   << " joint_state: " << joint_state_);
  //We do not call this function, as we want to listen to a different topic type
  //after_init();
  sub_command_ = node_.subscribe<sr_robot_msgs::JointMuscleValveControllerCommand>("command", 1, &SrhJointMuscleValveController::setCommandCB, this);
  return true;
}

void SrhJointMuscleValveController::starting(const ros::Time& time)
{
  command_ = 0.0;
  read_parameters();
}

/*
  bool SrhJointMuscleValveController::setGains(sr_robot_msgs::SetEffortControllerGains::Request &req,
                                          sr_robot_msgs::SetEffortControllerGains::Response &resp)
  {
    max_force_demand = req.max_force;
    friction_deadband = req.friction_deadband;

    //Setting the new parameters in the parameter server
    node_.setParam("max_force", max_force_demand);
    node_.setParam("friction_deadband", friction_deadband);

    return true;
  }
 */
bool SrhJointMuscleValveController::resetGains(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
  command_ = 0.0;

  read_parameters();

  if (has_j2)
    ROS_WARN_STREAM("Reseting controller gains: " << joint_state_->joint_->name << " and " << joint_state_2->joint_->name);
  else
    ROS_WARN_STREAM("Reseting controller gains: " << joint_state_->joint_->name);

  return true;
}

void SrhJointMuscleValveController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
}

void SrhJointMuscleValveController::update(const ros::Time& time, const ros::Duration& period)
{
  //The valve commands can have values between -4 and 4
  int8_t valve[2];
  unsigned int i;

  ROS_ASSERT(robot_ != NULL);
  ROS_ASSERT(joint_state_->joint_);

  if (!initialized_)
  {
    initialized_ = true;
    cmd_valve_muscle_[0] = 0.0;
    cmd_valve_muscle_[1] = 0.0;
    cmd_duration_ms_[0] = 0;
    cmd_duration_ms_[1] = 0;
    current_duration_ms_[0] = cmd_duration_ms_[0];
    current_duration_ms_[1] = cmd_duration_ms_[1];
  }


  //IGNORE the following  lines if we don't want to use the pressure sensors data
  //We don't want to define a modified version of JointState, as that would imply using a modified version of robot_state.hpp, controller manager,
  //ethercat_hardware and ros_etherCAT main loop
  // So we heve encoded the two uint16 that contain the data from the muscle pressure sensors into the double effort_. (We don't
  // have any measured effort in the muscle hand anyway).
  // Here we extract the pressure values from joint_state_->effort_ and decode that back into uint16.
  double pressure_0_tmp = fmod(joint_state_->effort_, 0x10000);
  double pressure_1_tmp = (fmod(joint_state_->effort_, 0x100000000) - pressure_0_tmp) / 0x10000;
  uint16_t pressure_0 = static_cast<uint16_t> (pressure_0_tmp + 0.5);
  uint16_t pressure_1 = static_cast<uint16_t> (pressure_1_tmp + 0.5);

  //****************************************





  //************************************************
  // Here goes the control algorithm

  // This controller will allow the user to specify a separate command for each of the two muscles that control the joint.
  // The user will also specify a duration in ms for that command. During this duration the command will be sent to the hand
  // every ms (every cycle of this 1Khz control loop).
  //Once this duration period has elapsed, a command of 0 will be sent to the muscle (meaning both the filling and emptying valves for that
  // muscle remain closed)
  // A duration of 0 means that there is no timeout, so the valve command will be sent to the muscle until a different valve command is received
  // BE CAREFUL WHEN USING A DURATION OF 0 AS THIS COULD EVENTUALLY DAMAGE THE MUSCLE

  for (i = 0; i < 2; ++i)
  {
    if (cmd_duration_ms_[i] == 0) // if the commanded duration is 0 it means that it will not timeout
    {
      // So we will use the last commanded valve command
      valve[i] = cmd_valve_muscle_[i];
    }
    else
    {
      if (current_duration_ms_[i] > 0) // If the command has not timed out yet
      {
        // we will use the last commanded valve command
        valve[i] = cmd_valve_muscle_[i];
        // and decrement the counter. This is a milliseconds counter, and this loop is running once every millisecond
        current_duration_ms_[i]--;
      }
      else // If the command has already timed out
      {
        // we will use 0 to close the valves
        valve[i] = 0;
      }
    }
  }


  //************************************************







  //************************************************
  // After doing any computation we consider, we encode the obtained valve commands into joint_state_->commanded_effort_
  //We don't want to define a modified version of JointState, as that would imply using a modified version of robot_state.hpp, controller manager,
  //ethercat_hardware and ros_etherCAT main loop
  // So the controller encodes the two int8 (that are in fact int4) that contain the valve commands into the double commanded_effort_. (We don't
  // have any real commanded_effort_ in the muscle hand anyway).

  uint16_t valve_tmp[2];
  for (i = 0; i < 2; ++i)
  {
    //Check that the limits of the valve command are not exceded
    if (valve[i] > 4)
      valve[i] = 4;
    if (valve[i] < -4)
      valve[i] = -4;
    //encode
    if (valve[i] < 0)
      valve_tmp[i] = -valve[i] + 8;
    else
      valve_tmp[i] = valve[i];
  }

  //We encode the valve 0 command in the lowest "half byte" i.e. the lowest 16 integer values in the double var (see decoding in simple_transmission_for_muscle.cpp)
  //the valve 1 command is envoded in the next 4 bits
  joint_state_->commanded_effort_ = static_cast<double> (valve_tmp[0]) + static_cast<double> (valve_tmp[1] << 4);

  //*******************************************************************************




  if (loop_count_ % 10 == 0)
  {
    if (controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;

      controller_state_publisher_->msg_.set_valve_muscle_0 = cmd_valve_muscle_[0];
      controller_state_publisher_->msg_.set_valve_muscle_1 = cmd_valve_muscle_[1];
      controller_state_publisher_->msg_.set_duration_muscle_0 = cmd_duration_ms_[0];
      controller_state_publisher_->msg_.set_duration_muscle_1 = cmd_duration_ms_[1];
      controller_state_publisher_->msg_.current_valve_muscle_0 = valve[0];
      controller_state_publisher_->msg_.current_valve_muscle_1 = valve[1];
      controller_state_publisher_->msg_.current_duration_muscle_0 = current_duration_ms_[0];
      controller_state_publisher_->msg_.current_duration_muscle_1 = current_duration_ms_[1];
      controller_state_publisher_->msg_.packed_valve = joint_state_->commanded_effort_;
      controller_state_publisher_->msg_.muscle_pressure_0 = pressure_0;
      controller_state_publisher_->msg_.muscle_pressure_1 = pressure_1;
      controller_state_publisher_->msg_.time_step = period.toSec();

      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;
}

void SrhJointMuscleValveController::read_parameters()
{

}

void SrhJointMuscleValveController::setCommandCB(const sr_robot_msgs::JointMuscleValveControllerCommandConstPtr& msg)
{
  cmd_valve_muscle_[0] = clamp_command(msg->cmd_valve_muscle[0]);
  cmd_valve_muscle_[1] = clamp_command(msg->cmd_valve_muscle[1]);
  //These variables hold the commanded duration
  cmd_duration_ms_[0] = static_cast<unsigned int> (msg->cmd_duration_ms[0]);
  cmd_duration_ms_[1] = static_cast<unsigned int> (msg->cmd_duration_ms[1]);
  //These are the actual counters that we will decrement
  current_duration_ms_[0] = cmd_duration_ms_[0];
  current_duration_ms_[1] = cmd_duration_ms_[1];
}

/// enforce that the value of the received command is in the allowed range

int8_t SrhJointMuscleValveController::clamp_command(int8_t cmd)
{
  if (cmd < cmd_valve_muscle_min_)
    return cmd_valve_muscle_min_;

  if (cmd > cmd_valve_muscle_max_)
    return cmd_valve_muscle_max_;

  return cmd;
}
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */

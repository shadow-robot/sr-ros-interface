/**
 * @file   srh_joint_position_controller.cpp
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
 * @brief  Follows a position target. The position demand is converted into a force
 * demand by a PID loop.
 *
 */

#include "sr_mechanism_controllers/srh_muscle_joint_position_controller.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"
#include <sstream>
#include <math.h>
#include "sr_utilities/sr_math_utils.hpp"

#include <std_msgs/Float64.h>

PLUGINLIB_EXPORT_CLASS(controller::SrhMuscleJointPositionController, controller_interface::ControllerBase)

using namespace std;

namespace controller
{

SrhMuscleJointPositionController::SrhMuscleJointPositionController()
  : position_deadband(0.015), command_acc_(0)
{
}

bool SrhMuscleJointPositionController::init(ros_ethercat_model::RobotState *robot, ros::NodeHandle &n)
{
  ROS_ASSERT(robot);
  robot_ = robot;
  node_ = n;

  if (!node_.getParam("joint", joint_name_))
  {
    ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }

  pid_controller_position_.reset(new control_toolbox::Pid());
  if (!pid_controller_position_->init(ros::NodeHandle(node_, "pid")))
    return false;

  controller_state_publisher_.reset(
                                    new realtime_tools::RealtimePublisher<sr_robot_msgs::JointMusclePositionControllerState>
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
      ROS_ERROR("SrhMuscleJointPositionController could not find the first joint relevant to \"%s\"\n", joint_name_.c_str());
      return false;
    }
    if (!joint_state_2)
    {
      ROS_ERROR("SrhMuscleJointPositionController could not find the second joint relevant to \"%s\"\n", joint_name_.c_str());
      return false;
    }
  }
  else
  {
    joint_state_ = robot_->getJointState(joint_name_);
    if (!joint_state_)
    {
      ROS_ERROR("SrhMuscleJointPositionController could not find joint named \"%s\"\n", joint_name_.c_str());
      return false;
    }
  }

  //get the min and max value for the current joint:
  get_min_max(robot_->robot_model_, joint_name_);

  friction_compensator.reset(new sr_friction_compensation::SrFrictionCompensator(joint_name_));

  serve_set_gains_ = node_.advertiseService("set_gains", &SrhMuscleJointPositionController::setGains, this);
  serve_reset_gains_ = node_.advertiseService("reset_gains", &SrhMuscleJointPositionController::resetGains, this);

  after_init();
  return true;
}

void SrhMuscleJointPositionController::starting(const ros::Time& time)
{
  resetJointState();
  pid_controller_position_->reset();
  read_parameters();

  if (has_j2)
    ROS_WARN_STREAM("Reseting PID for joints " << joint_state_->joint_->name << " and " << joint_state_2->joint_->name);
  else
    ROS_WARN_STREAM("Reseting PID for joint  " << joint_state_->joint_->name);
}

bool SrhMuscleJointPositionController::setGains(sr_robot_msgs::SetPidGains::Request &req,
                                                sr_robot_msgs::SetPidGains::Response &resp)
{
  ROS_INFO_STREAM("Setting new PID parameters. P:" << req.p << " / I:" << req.i <<
                  " / D:" << req.d << " / IClamp:" << req.i_clamp << ", max force: " << req.max_force << ", friction deadband: " << req.friction_deadband << " pos deadband: " << req.deadband);
  pid_controller_position_->setGains(req.p, req.i, req.d, req.i_clamp, -req.i_clamp);
  max_force_demand = req.max_force;
  friction_deadband = req.friction_deadband;
  position_deadband = req.deadband;

  //Setting the new parameters in the parameter server
  node_.setParam("pid/p", req.p);
  node_.setParam("pid/i", req.i);
  node_.setParam("pid/d", req.d);
  node_.setParam("pid/i_clamp", req.i_clamp);
  node_.setParam("pid/max_force", max_force_demand);
  node_.setParam("pid/position_deadband", position_deadband);
  node_.setParam("pid/friction_deadband", friction_deadband);

  return true;
}

bool SrhMuscleJointPositionController::resetGains(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
  resetJointState();

  if (!pid_controller_position_->init(ros::NodeHandle(node_, "pid")))
    return false;

  read_parameters();

  if (has_j2)
    ROS_WARN_STREAM("Reseting controller gains: " << joint_state_->joint_->name << " and " << joint_state_2->joint_->name);
  else
    ROS_WARN_STREAM("Reseting controller gains: " << joint_state_->joint_->name);

  return true;
}

void SrhMuscleJointPositionController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_position_->getGains(p, i, d, i_max, i_min);
}

void SrhMuscleJointPositionController::update(const ros::Time& time, const ros::Duration& period)
{
  //The valve commands can have values between -4 and 4
  int8_t valve[2];

  ROS_ASSERT(robot_ != NULL);
  ROS_ASSERT(joint_state_->joint_);

  if (!initialized_)
  {
    resetJointState();
    initialized_ = true;
  }
  if (has_j2)
    command_ = joint_state_->commanded_position_ + joint_state_2->commanded_position_;
  else
    command_ = joint_state_->commanded_position_;
  command_ = clamp_command(command_);

  //IGNORE the following  lines if we don't want to use the pressure sensors data
  //We don't want to define a modified version of JointState, as that would imply using a modified version of robot_state.hpp, controller manager,
  //ethercat_hardware and ros_etherCAT main loop
  // So we have encoded the two uint16 that contain the data from the muscle pressure sensors into the double effort_. (We don't
  // have any measured effort in the muscle hand anyway).
  // Here we extract the pressure values from joint_state_->effort_ and decode that back into uint16.
  double pressure_0_tmp = fmod(joint_state_->effort_, 0x10000);
  double pressure_1_tmp = (fmod(joint_state_->effort_, 0x100000000) - pressure_0_tmp) / 0x10000;
  uint16_t pressure_0 = static_cast<uint16_t> (pressure_0_tmp + 0.5);
  uint16_t pressure_1 = static_cast<uint16_t> (pressure_1_tmp + 0.5);

  //****************************************

  command_ = clamp_command(command_);

  //Compute position demand from position error:
  double error_position = 0.0;

  if (has_j2)
    error_position = (joint_state_->position_ + joint_state_2->position_) - command_;
  else
    error_position = joint_state_->position_ - command_;

  bool in_deadband = hysteresis_deadband.is_in_deadband(command_, error_position, position_deadband);

  //don't compute the error if we're in the deadband.
  if (in_deadband)
    error_position = 0.0;

  //Run the PID loop to get a new command, we don't do this at the full rate
  //as that will drive the valves too hard with switch changes. Instead we
  //store a longer time in the command accumulator to keep using at the full
  //loop rate.
  if (loop_count_ % 50 == 0)
  {
    double commanded_effort = pid_controller_position_->computeCommand(-error_position, period);

    //clamp the result to max force
    commanded_effort = min(commanded_effort, max_force_demand);
    commanded_effort = max(commanded_effort, -max_force_demand);

    if (!in_deadband)
    {
      if (has_j2)
        commanded_effort += friction_compensator->friction_compensation(joint_state_->position_ + joint_state_2->position_, joint_state_->velocity_ + joint_state_2->velocity_, int(commanded_effort), friction_deadband);
      else
        commanded_effort += friction_compensator->friction_compensation(joint_state_->position_, joint_state_->velocity_, int(commanded_effort), friction_deadband);
    }

    command_acc_ = commanded_effort;
  }

  // Drive the joint from the accumulator. This runs at full update speed so
  // we can keep valves open continuously (with high enough P). A value of 4 fills
  // the valve for the whole of the next 1ms. 2 for half that time etc. Negative
  // values empty the valve.
  // The 2 involved muscles will act complementary for the moment, when one inflates,
  // the other deflates at the same rate
  double amt = abs(command_acc_) < 4 ? fabs(command_acc_) : 4;
  if (abs(command_acc_) == 0)
  {
    valve[0] = 0;
    valve[1] = 0;
  }
  else if (command_acc_ > 0)
  {
    command_acc_ -= amt;
    valve[0] = amt;
    valve[1] = -amt;
  }
  else
  {
    command_acc_ += amt;
    valve[0] = -amt;
    valve[1] = amt;
  }


  //************************************************
  // After doing any computation we consider, we encode the obtained valve commands into joint_state_->commanded_effort_
  //We don't want to define a modified version of JointState, as that would imply using a modified version of robot_state.hpp, controller manager,
  //ethercat_hardware and ros_etherCAT main loop
  // So the controller encodes the two int8 (that are in fact int4) that contain the valve commands into the double commanded_effort_. (We don't
  // have any real commanded_effort_ in the muscle hand anyway).

  uint16_t valve_tmp[2];
  for (int i = 0; i < 2; ++i)
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
  //the valve 1 command is encoded in the next 4 bits
  joint_state_->commanded_effort_ = static_cast<double> (valve_tmp[0]) + static_cast<double> (valve_tmp[1] << 4);

  //*******************************************************************************

  // Send status msg
  if (loop_count_ % 10 == 0)
  {
    if (controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = command_;
      if (has_j2)
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
      controller_state_publisher_->msg_.pseudo_command = command_acc_;
      controller_state_publisher_->msg_.valve_muscle_0 = valve[0];
      controller_state_publisher_->msg_.valve_muscle_1 = valve[1];
      controller_state_publisher_->msg_.packed_valve = joint_state_->commanded_effort_;
      controller_state_publisher_->msg_.muscle_pressure_0 = pressure_0;
      controller_state_publisher_->msg_.muscle_pressure_1 = pressure_1;


      double dummy;
      getGains(controller_state_publisher_->msg_.p,
               controller_state_publisher_->msg_.i,
               controller_state_publisher_->msg_.d,
               controller_state_publisher_->msg_.i_clamp,
               dummy);
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;
}

void SrhMuscleJointPositionController::read_parameters()
{
  node_.param<double>("pid/max_force", max_force_demand, 1023.0);
  node_.param<double>("pid/position_deadband", position_deadband, 0.015);
  node_.param<int>("pid/friction_deadband", friction_deadband, 5);
}

void SrhMuscleJointPositionController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  joint_state_->commanded_position_ = msg->data;
  if (has_j2)
    joint_state_2->commanded_position_ = 0.0;
}

void SrhMuscleJointPositionController::resetJointState()
{
  if (has_j2)
  {
    joint_state_->commanded_position_ = joint_state_->position_;
    joint_state_2->commanded_position_ = joint_state_2->position_;
    command_ = joint_state_->position_ + joint_state_2->position_;
  }
  else
  {
    joint_state_->commanded_position_ = joint_state_->position_;
    command_ = joint_state_->position_;
  }
}
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */



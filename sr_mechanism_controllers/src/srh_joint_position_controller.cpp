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

#include "sr_mechanism_controllers/srh_joint_position_controller.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"
#include <sstream>
#include <math.h>
#include "sr_utilities/sr_math_utils.hpp"

#include <std_msgs/Float64.h>

PLUGINLIB_EXPORT_CLASS(controller::SrhJointPositionController, controller_interface::ControllerBase)

using namespace std;

namespace controller
{

SrhJointPositionController::SrhJointPositionController()
  : position_deadband(0.015)
{
}

bool SrhJointPositionController::init(ros_ethercat_model::RobotState *robot, ros::NodeHandle &n)
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

  controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
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
      ROS_ERROR("SrhJointPositionController could not find the first joint relevant to \"%s\"\n", joint_name_.c_str());
      return false;
    }
    if (!joint_state_2)
    {
      ROS_ERROR("SrhJointPositionController could not find the second joint relevant to \"%s\"\n", joint_name_.c_str());
      return false;
    }
    if (!joint_state_2->calibrated_)
    {
      ROS_ERROR("Joint %s not calibrated for SrhJointPositionController", joint_name_.c_str());
      return false;
    }
    else
      joint_state_->calibrated_ = true;
  }
  else
  {
    joint_state_ = robot_->getJointState(joint_name_);
    if (!joint_state_)
    {
      ROS_ERROR("SrhJointPositionController could not find joint named \"%s\"\n", joint_name_.c_str());
      return false;
    }
    if (!joint_state_->calibrated_)
    {
      ROS_ERROR("Joint %s not calibrated for SrhJointPositionController", joint_name_.c_str());
      return false;
    }
  }

  //get the min and max value for the current joint:
  get_min_max(robot_->robot_model_, joint_name_);

  friction_compensator.reset(new sr_friction_compensation::SrFrictionCompensator(joint_name_));

  serve_set_gains_ = node_.advertiseService("set_gains", &SrhJointPositionController::setGains, this);
  serve_reset_gains_ = node_.advertiseService("reset_gains", &SrhJointPositionController::resetGains, this);

  after_init();
  return true;
}

void SrhJointPositionController::starting(const ros::Time& time)
{
  resetJointState();
  pid_controller_position_->reset();
  read_parameters();

  if (has_j2)
    ROS_WARN_STREAM("Reseting PID for joints " << joint_state_->joint_->name << " and " << joint_state_2->joint_->name);
  else
    ROS_WARN_STREAM("Reseting PID for joint  " << joint_state_->joint_->name);
}

bool SrhJointPositionController::setGains(sr_robot_msgs::SetPidGains::Request &req,
                                          sr_robot_msgs::SetPidGains::Response &resp)
{
  ROS_INFO_STREAM("Setting new PID parameters. P:" << req.p << " / I:" << req.i <<
                  " / D:" << req.d << " / IClamp:" << req.i_clamp << ", max force: " <<
                  req.max_force << ", friction deadband: " << req.friction_deadband <<
                  " pos deadband: " << req.deadband);

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

bool SrhJointPositionController::resetGains(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
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

void SrhJointPositionController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_position_->getGains(p, i, d, i_max, i_min);
}

void SrhJointPositionController::update(const ros::Time& time, const ros::Duration& period)
{
  if (!has_j2 && !joint_state_->calibrated_)
    return;

  ROS_ASSERT(robot_);
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

  //Compute position demand from position error:
  double error_position = 0.0;
  double commanded_effort = 0.0;

  if (has_j2)
    error_position = (joint_state_->position_ + joint_state_2->position_) - command_;
  else
    error_position = joint_state_->position_ - command_;

  bool in_deadband = hysteresis_deadband.is_in_deadband(command_, error_position, position_deadband);

  //don't compute the error if we're in the deadband.
  if (in_deadband)
    error_position = 0.0;

  commanded_effort = pid_controller_position_->computeCommand(-error_position, period);

  //clamp the result to max force
  commanded_effort = min(commanded_effort, (max_force_demand * max_force_factor_));
  commanded_effort = max(commanded_effort, -(max_force_demand * max_force_factor_));

  if (!in_deadband)
  {
    if (has_j2)
      commanded_effort += friction_compensator->friction_compensation(joint_state_->position_ + joint_state_2->position_,
                                                                      joint_state_->velocity_ + joint_state_2->velocity_,
                                                                      int(commanded_effort),
                                                                      friction_deadband);
    else
      commanded_effort += friction_compensator->friction_compensation(joint_state_->position_,
                                                                      joint_state_->velocity_,
                                                                      int(commanded_effort),
                                                                      friction_deadband);
  }

  joint_state_->commanded_effort_ = commanded_effort;

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
      controller_state_publisher_->msg_.command = commanded_effort;

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

void SrhJointPositionController::read_parameters()
{
  node_.param<double>("pid/max_force", max_force_demand, 1023.0);
  node_.param<double>("pid/position_deadband", position_deadband, 0.015);
  node_.param<int>("pid/friction_deadband", friction_deadband, 5);
}

void SrhJointPositionController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  joint_state_->commanded_position_ = msg->data;
  if (has_j2)
    joint_state_2->commanded_position_ = 0.0;
}

void SrhJointPositionController::resetJointState()
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



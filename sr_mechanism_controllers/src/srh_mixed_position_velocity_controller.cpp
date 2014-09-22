/**
 * @file   srh_mixed_position_velocity_controller.cpp
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
 * @brief Compute a velocity demand from the position error using
 *        a PID loop.
 *        The velocity demand is then converted into a force demand by a
 *        second PID loop and is sent to the motor.
 *
 */

#include "sr_mechanism_controllers/srh_mixed_position_velocity_controller.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"
#include <sstream>
#include <math.h>
#include "sr_utilities/sr_math_utils.hpp"

#include <std_msgs/Float64.h>

PLUGINLIB_EXPORT_CLASS(controller::SrhMixedPositionVelocityJointController, controller_interface::ControllerBase)

using namespace std;

namespace controller
{

SrhMixedPositionVelocityJointController::SrhMixedPositionVelocityJointController()
  : max_velocity_(1.0), min_velocity_(-1.0),
  position_deadband(0.05), motor_min_force_threshold(0)
{
}

bool SrhMixedPositionVelocityJointController::init(ros_ethercat_model::RobotState *robot, ros::NodeHandle &n)
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
  if (!pid_controller_position_->init(ros::NodeHandle(node_, "position_pid")))
    return false;

  pid_controller_velocity_.reset(new control_toolbox::Pid());
  if (!pid_controller_velocity_->init(ros::NodeHandle(node_, "velocity_pid")))
    return false;

  controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<sr_robot_msgs::JointControllerState>
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
      ROS_ERROR("SrhMixedPositionVelocityController could not find the first joint relevant to \"%s\"\n", joint_name_.c_str());
      return false;
    }
    if (!joint_state_2)
    {
      ROS_ERROR("SrhMixedPositionVelocityController could not find the second joint relevant to \"%s\"\n", joint_name_.c_str());
      return false;
    }
    if (!joint_state_2->calibrated_)
    {
      ROS_ERROR("Joint %s not calibrated for SrhMixedPositionVelocityController", joint_name_.c_str());
      return false;
    }
    else
      joint_state_->calibrated_ = true;
  }
  else //"normal" joints
  {
    joint_state_ = robot_->getJointState(joint_name_);
    if (!joint_state_)
    {
      ROS_ERROR("SrhMixedPositionVelocityController could not find joint named \"%s\"\n",
                joint_name_.c_str());
      return false;
    }
    if (!joint_state_->calibrated_)
    {
      ROS_ERROR("Joint %s not calibrated for SrhMixedPositionVelocityJointController", joint_name_.c_str());
      return false;
    }
  }
  friction_compensator.reset(new sr_friction_compensation::SrFrictionCompensator(joint_name_));

  //get the min and max value for the current joint:
  get_min_max(robot_->robot_model_, joint_name_);

  serve_set_gains_ = node_.advertiseService("set_gains", &SrhMixedPositionVelocityJointController::setGains, this);
  serve_reset_gains_ = node_.advertiseService("reset_gains", &SrhMixedPositionVelocityJointController::resetGains, this);

  ROS_DEBUG_STREAM(" joint_state name: " << joint_state_->joint_->name);
  ROS_DEBUG_STREAM(" In Init: " << getJointName() << " This: " << this
                   << " joint_state: " << joint_state_);

#ifdef DEBUG_PUBLISHER
  if (string("FFJ3").compare(getJointName()) == 0)
  {
    ROS_INFO("Publishing debug info for FFJ3 mixed position/velocity controller");
    stringstream ss2;
    ss2 << getJointName() << "debug_velocity";
    debug_pub = n_tilde_.advertise<std_msgs::Float64>(ss2.str(), 2);
  }
#endif

  after_init();
  return true;
}

void SrhMixedPositionVelocityJointController::starting(const ros::Time& time)
{
  resetJointState();
  pid_controller_position_->reset();
  pid_controller_velocity_->reset();
  read_parameters();

  if (has_j2)
    ROS_WARN_STREAM("Reseting PID for joints " << joint_state_->joint_->name << " and " << joint_state_2->joint_->name);
  else
    ROS_WARN_STREAM("Reseting PID for joint  " << joint_state_->joint_->name);
}

bool SrhMixedPositionVelocityJointController::setGains(sr_robot_msgs::SetMixedPositionVelocityPidGains::Request &req,
                                                       sr_robot_msgs::SetMixedPositionVelocityPidGains::Response &resp)
{
  ROS_INFO_STREAM("New parameters: " << "PID pos: [" << req.position_p << ", " << req.position_i << ", " << req.position_d << ", " << req.position_i_clamp << "] PID vel: [" << req.velocity_p << ", " << req.velocity_i << ", " << req.velocity_d << ", " << req.velocity_i_clamp << "], max force: " << req.max_force << ", friction deadband: " << req.friction_deadband << " pos deadband: " << req.position_deadband << " min and max vel: [" << req.min_velocity << ", " << req.max_velocity << "]");

  pid_controller_position_->setGains(req.position_p, req.position_i, req.position_d, req.position_i_clamp, -req.position_i_clamp);

  pid_controller_velocity_->setGains(req.velocity_p, req.velocity_i, req.velocity_d, req.velocity_i_clamp, -req.velocity_i_clamp);
  max_force_demand = req.max_force;
  friction_deadband = req.friction_deadband;
  position_deadband = req.position_deadband;

  //setting the position controller parameters
  min_velocity_ = req.min_velocity;
  max_velocity_ = req.max_velocity;

  //Setting the new parameters in the parameter server
  node_.setParam("position_pid/p", req.position_p);
  node_.setParam("position_pid/i", req.position_i);
  node_.setParam("position_pid/d", req.position_d);
  node_.setParam("position_pid/i_clamp", req.position_i_clamp);

  node_.setParam("velocity_pid/p", req.velocity_p);
  node_.setParam("velocity_pid/i", req.velocity_i);
  node_.setParam("velocity_pid/d", req.velocity_d);
  node_.setParam("velocity_pid/i_clamp", req.velocity_i_clamp);

  node_.setParam("position_pid/min_velocity", min_velocity_);
  node_.setParam("position_pid/max_velocity", max_velocity_);
  node_.setParam("position_pid/position_deadband", position_deadband);

  node_.setParam("velocity_pid/friction_deadband", friction_deadband);
  node_.setParam("velocity_pid/max_force", max_force_demand);
  node_.setParam("motor_min_force_threshold", motor_min_force_threshold);

  return true;
}

bool SrhMixedPositionVelocityJointController::resetGains(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
  resetJointState();

  if (!pid_controller_position_->init(ros::NodeHandle(node_, "position_pid")))
    return false;

  if (!pid_controller_velocity_->init(ros::NodeHandle(node_, "velocity_pid")))
    return false;

  read_parameters();

  if (has_j2)
    ROS_WARN_STREAM("Reseting controller gains: " << joint_state_->joint_->name << " and " << joint_state_2->joint_->name);
  else
    ROS_WARN_STREAM("Reseting controller gains: " << joint_state_->joint_->name);

  return true;
}

void SrhMixedPositionVelocityJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_position_->getGains(p, i, d, i_max, i_min);
}

void SrhMixedPositionVelocityJointController::getGains_velocity(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_velocity_->getGains(p, i, d, i_max, i_min);
}

void SrhMixedPositionVelocityJointController::update(const ros::Time& time, const ros::Duration& period)
{
  if (!has_j2 &&!joint_state_->calibrated_)
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

  ////////////
  // POSITION
  /////
  //Compute velocity demand from position error:
  double error_position = 0.0;
  if (has_j2)
  {
    error_position = command_ - (joint_state_->position_ + joint_state_2->position_);
    ROS_DEBUG_STREAM("j0: " << joint_state_->position_ + joint_state_2->position_);
  }
  else
    error_position = command_ - joint_state_->position_;

  //are we in the deadband?
  bool in_deadband = hysteresis_deadband.is_in_deadband(command_, error_position, position_deadband);

  if (in_deadband) //consider the error as 0 if we're in the deadband
  {
    error_position = 0.0;
    pid_controller_position_->reset();
  }

  double commanded_velocity = 0.0;
  double error_velocity = 0.0;
  double commanded_effort = 0.0;

  //compute the velocity demand using the position pid loop
  commanded_velocity = pid_controller_position_->computeCommand(-error_position, period);
  //saturate the velocity demand
  commanded_velocity = max(commanded_velocity, min_velocity_);
  commanded_velocity = min(commanded_velocity, max_velocity_);

  ////////////
  // VELOCITY
  /////

  //velocity loop:
  if (!in_deadband) //don't compute the error if we're in the deadband
  {
    //we're not in the deadband, compute the error
    if (has_j2)
      error_velocity = commanded_velocity - (joint_state_->velocity_ + joint_state_2->velocity_);
    else
      error_velocity = commanded_velocity - joint_state_->velocity_;
  }
  commanded_effort = pid_controller_velocity_->computeCommand(-error_velocity, period);

  //clamp the result to max force
  commanded_effort = min(commanded_effort, (max_force_demand * max_force_factor_));
  commanded_effort = max(commanded_effort, -(max_force_demand * max_force_factor_));

  //Friction compensation, only if we're not in the deadband.
  int friction_offset = 0;
  if (!in_deadband)
  {
    if (has_j2)
      friction_offset = friction_compensator->friction_compensation(joint_state_->position_ + joint_state_2->position_, joint_state_->velocity_ + joint_state_2->velocity_, int(commanded_effort), friction_deadband);
    else
      friction_offset = friction_compensator->friction_compensation(joint_state_->position_, joint_state_->velocity_, int(commanded_effort), friction_deadband);

    commanded_effort += friction_offset;
  }

  //if the demand is too small to be executed by the motor, then we ask for a force
  // of 0
  if (fabs(commanded_effort) <= motor_min_force_threshold)
    commanded_effort = 0.0;

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
      controller_state_publisher_->msg_.commanded_velocity = commanded_velocity;

      controller_state_publisher_->msg_.error = error_position;
      controller_state_publisher_->msg_.time_step = period.toSec();

      controller_state_publisher_->msg_.command = commanded_effort;
      controller_state_publisher_->msg_.measured_effort = joint_state_->effort_;

      controller_state_publisher_->msg_.friction_compensation = friction_offset;

      double dummy;
      getGains(controller_state_publisher_->msg_.position_p,
               controller_state_publisher_->msg_.position_i,
               controller_state_publisher_->msg_.position_d,
               controller_state_publisher_->msg_.position_i_clamp,
               dummy);

      getGains_velocity(controller_state_publisher_->msg_.velocity_p,
                        controller_state_publisher_->msg_.velocity_i,
                        controller_state_publisher_->msg_.velocity_d,
                        controller_state_publisher_->msg_.velocity_i_clamp,
                        dummy);

      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;
}

void SrhMixedPositionVelocityJointController::read_parameters()
{
  node_.param<double>("position_pid/min_velocity", min_velocity_, -1.0);
  node_.param<double>("position_pid/max_velocity", max_velocity_, 1.0);
  node_.param<double>("position_pid/position_deadband", position_deadband, 0.015);

  node_.param<int>("velocity_pid/friction_deadband", friction_deadband, 5);
  node_.param<double>("velocity_pid/max_force", max_force_demand, 1023.0);
  node_.param<int>("motor_min_force_threshold", motor_min_force_threshold, 0);
}

void SrhMixedPositionVelocityJointController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  joint_state_->commanded_position_ = msg->data;
  if (has_j2)
    joint_state_2->commanded_position_ = 0.0;
}

void SrhMixedPositionVelocityJointController::resetJointState()
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



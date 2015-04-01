/**
 * @file   srh_joint_effort_controller.cpp
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

#include "sr_mechanism_controllers/srh_joint_effort_controller.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"
#include <sstream>
#include <math.h>
#include "sr_utilities/sr_math_utils.hpp"

#include <std_msgs/Float64.h>

PLUGINLIB_EXPORT_CLASS(controller::SrhEffortJointController, controller_interface::ControllerBase)

using namespace std;

namespace controller
{

bool SrhEffortJointController::init(ros_ethercat_model::RobotState *robot, ros::NodeHandle &n)
{
  ROS_ASSERT(robot);
  robot_ = robot;
  node_ = n;

  if (!node_.getParam("joint", joint_name_))
  {
    ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }

  pid_controller_effort_.reset(new control_toolbox::Pid());
  if (!pid_controller_effort_->init(ros::NodeHandle(node_, "pid")))
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
      ROS_ERROR("SrhEffortJointController could not find the first joint relevant to \"%s\"\n", joint_name_.c_str());
      return false;
    }
    if (!joint_state_2)
    {
      ROS_ERROR("SrhEffortJointController could not find the second joint relevant to \"%s\"\n", joint_name_.c_str());
      return false;
    }
  }
  else
  {
    joint_state_ = robot_->getJointState(joint_name_);
    if (!joint_state_)
    {
      ROS_ERROR("SrhEffortJointController could not find joint named \"%s\"\n", joint_name_.c_str());
      return false;
    }
  }

  friction_compensator.reset(new sr_friction_compensation::SrFrictionCompensator(joint_name_));

  serve_set_gains_ = node_.advertiseService("set_gains", &SrhEffortJointController::setGains, this);
  serve_reset_gains_ = node_.advertiseService("reset_gains", &SrhEffortJointController::resetGains, this);

  ROS_DEBUG_STREAM(" joint_state name: " << joint_state_->joint_->name);
  ROS_DEBUG_STREAM(" In Init: " << getJointName() << " This: " << this
                   << " joint_state: " << joint_state_);

  after_init();
  return true;
}

void SrhEffortJointController::starting(const ros::Time& time)
{
  resetJointState();
  pid_controller_effort_->reset();
  read_parameters();
}

bool SrhEffortJointController::setGains(sr_robot_msgs::SetPidGains::Request &req,
					sr_robot_msgs::SetPidGains::Response &resp)
{
  ROS_INFO_STREAM("Setting new PID parameters. P:" << req.p << " / I:" << req.i <<
                  " / D:" << req.d << " / IClamp:" << req.i_clamp << ", max force: " <<
                  req.max_force << ", friction deadband: " << req.friction_deadband <<
                  " torque deadband: " << req.deadband);

  pid_controller_effort_->setGains(req.p, req.i, req.d, req.i_clamp, -req.i_clamp);
  max_force_demand = req.max_force;
  friction_deadband = req.friction_deadband;
  effort_deadband = req.deadband;

  //Setting the new parameters in the parameter server
  node_.setParam("pid/p", req.p);
  node_.setParam("pid/i", req.i);
  node_.setParam("pid/d", req.d);
  node_.setParam("pid/i_clamp", req.i_clamp);
  node_.setParam("pid/max_force", max_force_demand);
  node_.setParam("pid/effort_deadband", effort_deadband);
  node_.setParam("pid/friction_deadband", friction_deadband);

  return true;
}

bool SrhEffortJointController::resetGains(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
  resetJointState();

  if (!pid_controller_effort_->init(ros::NodeHandle(node_, "pid")))
    return false;

  read_parameters();

  if (has_j2)
    ROS_WARN_STREAM("Reseting controller gains: " << joint_state_->joint_->name << " and " << joint_state_2->joint_->name);
  else
    ROS_WARN_STREAM("Reseting controller gains: " << joint_state_->joint_->name);

  return true;
}

void SrhEffortJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
{
  pid_controller_effort_->getGains(p, i, d, i_max, i_min);
}

void SrhEffortJointController::update(const ros::Time& time, const ros::Duration& period)
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

  double commanded_effort = command_;

  //check if we're running the torque control on the motor board or not
  if(joint_state_->control_type_.control_type == sr_robot_msgs::ControlType::FORCE)
  {
    //Computes the PWM command to be sent to the motor.
    // The torque pid loop is running on the host.
    double error_effort = 0.0;

    if (has_j2)
      error_effort = (joint_state_->effort_ + joint_state_2->effort_) - command_;
    else
      error_effort = joint_state_->effort_ - command_;

    bool in_deadband = hysteresis_deadband.is_in_deadband(command_, error_effort, effort_deadband);
    commanded_effort = pid_controller_effort_->computeCommand(-error_effort, period);
  }

  //Clamps the effort
  commanded_effort = min(commanded_effort, (max_force_demand * max_force_factor_));
  commanded_effort = max(commanded_effort, -(max_force_demand * max_force_factor_));

  //Friction compensation
  if (has_j2)
    commanded_effort += friction_compensator->friction_compensation(joint_state_->position_ + joint_state_2->position_, joint_state_->velocity_ + joint_state_2->velocity_, int(commanded_effort), friction_deadband);
  else
    commanded_effort += friction_compensator->friction_compensation(joint_state_->position_, joint_state_->velocity_, int(commanded_effort), friction_deadband);

  joint_state_->commanded_effort_ = commanded_effort;

  if (loop_count_ % 10 == 0)
  {
    if (controller_state_publisher_ && controller_state_publisher_->trylock())
    {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = command_;
      controller_state_publisher_->msg_.process_value = joint_state_->effort_;
      //TODO: compute the derivative of the effort.
      controller_state_publisher_->msg_.process_value_dot = -1.0;
      controller_state_publisher_->msg_.error = commanded_effort - joint_state_->effort_;
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

void SrhEffortJointController::read_parameters()
{
  node_.param<double>("max_force", max_force_demand, 1023.0);
  node_.param<int>("friction_deadband", friction_deadband, 5);
}

void SrhEffortJointController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
{
  command_ = msg->data;
}

void SrhEffortJointController::resetJointState()
{
  if (has_j2)
  {
    joint_state_->commanded_effort_ = joint_state_->effort_;
    joint_state_2->commanded_effort_ = joint_state_2->effort_;
    command_ = joint_state_->effort_ + joint_state_2->effort_;
  }
  else
  {
    joint_state_->commanded_effort_ = joint_state_->effort_;
    command_ = joint_state_->effort_;
  }
}

}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
 */

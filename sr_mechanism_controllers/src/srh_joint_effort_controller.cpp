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

PLUGINLIB_EXPORT_CLASS( controller::SrhEffortJointController, controller_interface::ControllerBase)

using namespace std;

namespace controller {

  bool SrhEffortJointController::init(ros_ethercat_model::RobotState *robot, const string &joint_name)
  {
    ROS_DEBUG(" --------- ");
    ROS_DEBUG_STREAM("Init: " << joint_name);

    ROS_ASSERT(robot);
    robot_ = robot;

    //joint 0s
    if (joint_name[3] == '0')
    {
      has_j2 = true;
      string j1 = joint_name.substr(0,3) + "1";
      string j2 = joint_name.substr(0,3) + "2";
      ROS_DEBUG_STREAM("Joint 0: " << j1 << " " << j2);

      joint_state_ = robot_->getJointState(j1);
      if (!joint_state_)
      {
        ROS_ERROR("SrhEffortJointController could not find joint named \"%s\"\n",
                  j1.c_str());
        return false;
      }

      joint_state_2 = robot_->getJointState(j2);
      if (!joint_state_2)
      {
        ROS_ERROR("SrhEffortJointController could not find joint named \"%s\"\n",
                  j2.c_str());
        return false;
      }
    }
    else
    {
      has_j2 = false;
      joint_state_ = robot_->getJointState(joint_name);
      if (!joint_state_)
      {
        ROS_ERROR("SrhJointPositionController could not find joint named \"%s\"\n",
                  joint_name.c_str());
        return false;
      }
    }

    friction_compensator = boost::shared_ptr<sr_friction_compensation::SrFrictionCompensator>(new sr_friction_compensation::SrFrictionCompensator(joint_name));

    serve_set_gains_ = node_.advertiseService("set_gains", &SrhEffortJointController::setGains, this);
    serve_reset_gains_ = node_.advertiseService("reset_gains", &SrhEffortJointController::resetGains, this);

    ROS_DEBUG_STREAM(" joint_state name: " << joint_state_->joint_->name);
    ROS_DEBUG_STREAM(" In Init: " << getJointName() << " This: " << this
                     << " joint_state: "<<joint_state_ );

    after_init();
    return true;
  }

  bool SrhEffortJointController::init(ros_ethercat_model::RobotState *robot, ros::NodeHandle &n)
  {
    ROS_ASSERT(robot);
    node_ = n;

    string joint_name;
    if (!node_.getParam("joint", joint_name)) {
      ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }

    controller_state_publisher_.reset(
      new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
      (node_, "state", 1));

    return init(robot, joint_name);
  }


  void SrhEffortJointController::starting(const ros::Time& time)
  {
    command_ = 0.0;
    read_parameters();
  }

  bool SrhEffortJointController::setGains(sr_robot_msgs::SetEffortControllerGains::Request &req,
                                          sr_robot_msgs::SetEffortControllerGains::Response &resp)
  {
    max_force_demand = req.max_force;
    friction_deadband = req.friction_deadband;

    //Setting the new parameters in the parameter server
    node_.setParam("max_force", max_force_demand);
    node_.setParam("friction_deadband", friction_deadband);

    return true;
  }

  bool SrhEffortJointController::resetGains(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
  {
    command_ = 0.0;

    read_parameters();

    if( has_j2 )
      ROS_WARN_STREAM("Reseting controller gains: " << joint_state_->joint_->name << " and " << joint_state_2->joint_->name);
    else
      ROS_WARN_STREAM("Reseting controller gains: " << joint_state_->joint_->name);

    return true;
  }

  void SrhEffortJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
  {
  }

  void SrhEffortJointController::update(const ros::Time& time, const ros::Duration& period)
  {
    if( !has_j2)
    {
      if (!joint_state_->calibrated_)
        return;
    }

    ROS_ASSERT(robot_ != NULL);
    ROS_ASSERT(joint_state_->joint_);

    if (!initialized_)
    {
      initialized_ = true;
      command_ = 0.0;
    }

    //The commanded effort is the error directly:
    // the PID loop for the force controller is running on the
    // motorboard.
    double commanded_effort = command_;

    //Clamps the effort
    commanded_effort = min( commanded_effort, (max_force_demand * max_force_factor_) );
    commanded_effort = max( commanded_effort, -(max_force_demand * max_force_factor_) );

    //Friction compensation
    if( has_j2 )
      commanded_effort += friction_compensator->friction_compensation( joint_state_->position_ + joint_state_2->position_ , joint_state_->velocity_ + joint_state_2->velocity_, int(commanded_effort), friction_deadband );
    else
      commanded_effort += friction_compensator->friction_compensation( joint_state_->position_ , joint_state_->velocity_, int(commanded_effort), friction_deadband );

    joint_state_->commanded_effort_ = commanded_effort;

    if(loop_count_ % 10 == 0)
    {
      if(controller_state_publisher_ && controller_state_publisher_->trylock())
      {
        controller_state_publisher_->msg_.header.stamp = time;
        controller_state_publisher_->msg_.set_point = command_;
        controller_state_publisher_->msg_.process_value = joint_state_->measured_effort_;
        //TODO: compute the derivative of the effort.
        controller_state_publisher_->msg_.process_value_dot = -1.0;
        controller_state_publisher_->msg_.error = commanded_effort - joint_state_->measured_effort_;
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

}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

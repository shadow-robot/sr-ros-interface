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
 * @brief Compute a velocity demand from the position error:
 *  we use this function (velocity_demand = f(position_error))
 *  to converge smoothly on the position we want.
 *       ____
 *      /
 *     /
 * ___/
 *
 * The velocity demand is then converted into a force demand by a
 * PID loop.
 *
 */

#include "sr_edc_mechanism_controllers/srh_mixed_position_velocity_controller.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"
#include <sstream>
#include <math.h>
#include "sr_utilities/sr_math_utils.hpp"

#include <std_msgs/Float64.h>

PLUGINLIB_DECLARE_CLASS(sr_edc_mechanism_controllers, SrhMixedPositionVelocityJointController, controller::SrhMixedPositionVelocityJointController, pr2_controller_interface::Controller)

using namespace std;

namespace controller {

  SrhMixedPositionVelocityJointController::SrhMixedPositionVelocityJointController()
    : joint_state_(NULL), command_(0),
      loop_count_(0),  initialized_(false), robot_(NULL), last_time_(0),
      n_tilde_("~"),
      max_velocity_(1.0), min_velocity_(-1.0), slope_velocity_(10.0),
      max_position_error_(0.0), min_position_error_(0.0),
      max_force_demand(1000.), position_deadband(0.05), friction_deadband(5)
  {
    set_min_max_position_errors_();
  }

  SrhMixedPositionVelocityJointController::~SrhMixedPositionVelocityJointController()
  {
    sub_command_.shutdown();
  }

  bool SrhMixedPositionVelocityJointController::init(pr2_mechanism_model::RobotState *robot, const std::string &joint_name,
                                                     const control_toolbox::Pid &pid_velocity)
  {
    ROS_DEBUG(" --------- ");
    ROS_DEBUG_STREAM("Init: " << joint_name);

    assert(robot);
    robot_ = robot;
    last_time_ = robot->getTime();

    joint_state_ = robot_->getJointState(joint_name);
    if (!joint_state_)
    {
      ROS_ERROR("SrhMixedPositionVelocityController could not find joint named \"%s\"\n",
                joint_name.c_str());
      return false;
    }
    if (!joint_state_->calibrated_)
    {
      ROS_ERROR("Joint %s not calibrated for SrhMixedPositionVelocityJointController", joint_name.c_str());
      return false;
    }

    friction_compensator = boost::shared_ptr<sr_friction_compensation::SrFrictionCompensator>(new sr_friction_compensation::SrFrictionCompensator(joint_name));

    pid_controller_velocity_ = pid_velocity;

    serve_set_gains_ = node_.advertiseService("set_gains", &SrhMixedPositionVelocityJointController::setGains, this);

    ROS_DEBUG_STREAM(" joint_state name: " << joint_state_->joint_->name);
    ROS_DEBUG_STREAM(" In Init: " << getJointName() << " This: " << this
                     << " joint_state: "<<joint_state_ );

    std::stringstream ss;
    ss << getJointName() << "/set_velocity";

    if( std::string("FFJ3").compare(getJointName()) == 0)
    {
      ROS_INFO("Publishing debug infor for FFJ3 mixed position/velocity controller");
      std::stringstream ss2;
      ss2 << getJointName() << "debug_velocity";
      debug_pub = n_tilde_.advertise<std_msgs::Float64>(ss2.str(), 2);
    }

    return true;
  }

  bool SrhMixedPositionVelocityJointController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
  {
    assert(robot);
    node_ = n;

    std::string joint_name;
    if (!node_.getParam("joint", joint_name)) {
      ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }

    control_toolbox::Pid pid_velocity;
    if (!pid_velocity.init(ros::NodeHandle(node_, "pid")))
      return false;


    controller_state_publisher_.reset(
      new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointControllerState>
      (node_, "state", 1));

    sub_command_ = node_.subscribe<std_msgs::Float64>("command", 1, &SrhMixedPositionVelocityJointController::setCommandCB, this);

    return init(robot, joint_name, pid_velocity);
  }


  void SrhMixedPositionVelocityJointController::starting()
  {
    command_ = joint_state_->position_;
    pid_controller_velocity_.reset();
    ROS_WARN("Reseting PID");
  }

  bool SrhMixedPositionVelocityJointController::setGains(sr_robot_msgs::SetMixedPositionVelocityPidGains::Request &req,
                                                         sr_robot_msgs::SetMixedPositionVelocityPidGains::Response &resp)
  {
    pid_controller_velocity_.setGains(req.p,req.i,req.d,req.i_clamp,-req.i_clamp);
    max_force_demand = req.max_force;
    friction_deadband = req.friction_deadband;
    position_deadband = req.position_deadband;

    //setting the position controller parameters
    min_velocity_ = req.min_velocity;
    max_velocity_ = req.max_velocity;
    slope_velocity_ = req.velocity_slope;

    if( slope_velocity_ != 0.0 )
    {
      set_min_max_position_errors_();
      return true;
    }

    min_velocity_ = -1.0;
    max_velocity_ = 1.0;
    slope_velocity_ = 1.0;
    set_min_max_position_errors_();
    return false;
  }

  void SrhMixedPositionVelocityJointController::getGains(double &p, double &i, double &d, double &i_max, double &i_min)
  {
    pid_controller_velocity_.getGains(p,i,d,i_max,i_min);
  }


  std::string SrhMixedPositionVelocityJointController::getJointName()
  {
    ROS_DEBUG_STREAM(" joint_state: "<<joint_state_ << " This: " << this);

    return joint_state_->joint_->name;
  }

// Set the joint position command
  void SrhMixedPositionVelocityJointController::setCommand(double cmd)
  {
    command_ = cmd;
  }

// Return the current position command
  void SrhMixedPositionVelocityJointController::getCommand(double & cmd)
  {
    cmd = command_;
  }

  void SrhMixedPositionVelocityJointController::update()
  {
    if (!joint_state_->calibrated_)
      return;

    assert(robot_ != NULL);
    ros::Time time = robot_->getTime();
    assert(joint_state_->joint_);
    dt_= time - last_time_;

    if (!initialized_)
    {
      initialized_ = true;
      command_ = joint_state_->position_;
    }

    //Compute velocity demand from position error:
    double error_position = joint_state_->position_ - command_;

    double commanded_velocity = 0.0;
    double error_velocity = 0.0;
    double commanded_effort = 0.0;

    //don't compute the error if we're in the deadband.
    if( !hysteresis_deadband.is_in_deadband(command_, error_position, position_deadband) )
    {
      //compute the velocity demand from the simple interpoler
      commanded_velocity = compute_velocity_demand(error_position);

      //velocity loop:
      error_velocity = joint_state_->velocity_ - commanded_velocity;
      commanded_effort = pid_controller_velocity_.updatePid(error_velocity, dt_);

      commanded_effort += joint_state_->commanded_effort_;

      //clamp the result to max force
      commanded_effort = min( commanded_effort, max_force_demand );
      commanded_effort = max( commanded_effort, -max_force_demand );

      //Friction compensation
      //if( std::string("FFJ3").compare( getJointName() ) == 0 )
      //  ROS_INFO_STREAM(getJointName() << ": before fc: velocity demand=" << commanded_velocity << " force demand=" << commanded_effort << " / error: " << error_velocity );
      commanded_effort += friction_compensator->friction_compensation( joint_state_->position_ , int(commanded_effort), friction_deadband );

      //if( std::string("FFJ3").compare( getJointName() ) == 0 )
      //  ROS_INFO_STREAM(getJointName() << ": after fc: effort=" << commanded_effort );
    }
    /*
      else
      {
      if( std::string("FFJ3").compare(getJointName()) == 0)
      ROS_ERROR("in deadband");
      }
    **/
    joint_state_->commanded_effort_ = commanded_effort;

    if(loop_count_ % 10 == 0)
    {
      if( std::string("FFJ3").compare(getJointName()) == 0)
      {
        std_msgs::Float64 msg;
        msg.data = commanded_velocity;
        debug_pub.publish(msg);
      }

      if(controller_state_publisher_ && controller_state_publisher_->trylock())
      {
        controller_state_publisher_->msg_.header.stamp = time;
        controller_state_publisher_->msg_.set_point = command_;
        controller_state_publisher_->msg_.process_value = joint_state_->position_;
        controller_state_publisher_->msg_.process_value_dot = joint_state_->velocity_;
        controller_state_publisher_->msg_.error = error_velocity;
        controller_state_publisher_->msg_.time_step = dt_.toSec();
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

    last_time_ = time;
  }

  void SrhMixedPositionVelocityJointController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
  {
    command_ = msg->data;
  }

  double SrhMixedPositionVelocityJointController::compute_velocity_demand(double position_error)
  {
    if( position_error < min_position_error_ )
      return min_velocity_;

    if( position_error > max_position_error_ )
      return max_velocity_;

    return sr_math_utils::linear_interpolate_(position_error,
                                              min_position_error_, min_velocity_,
                                              max_position_error_, max_velocity_);
  }

  void SrhMixedPositionVelocityJointController::set_min_max_position_errors_()
  {
    //Because the slope goes through (0,0), we have:
    // velocity = slope_velocity_ * position_error
    min_position_error_ = min_velocity_ / slope_velocity_;
    max_position_error_ = max_velocity_ / slope_velocity_;
  }

}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/



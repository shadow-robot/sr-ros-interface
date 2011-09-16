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

#include "sr_mechanism_controllers/srh_mixed_position_velocity_controller.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"
#include <sstream>
#include <math.h>
#include "sr_utilities/sr_math_utils.hpp"

#include <std_msgs/Float64.h>

PLUGINLIB_DECLARE_CLASS(sr_mechanism_controllers, SrhMixedPositionVelocityJointController, controller::SrhMixedPositionVelocityJointController, pr2_controller_interface::Controller)

using namespace std;

namespace controller {

  SrhMixedPositionVelocityJointController::SrhMixedPositionVelocityJointController()
    : SrController(), max_velocity_(1.0), min_velocity_(-1.0), slope_velocity_(20.0),
      max_position_error_(0.0), min_position_error_(0.0),
      position_deadband(0.05)
  {
    set_min_max_position_errors_();
  }

  SrhMixedPositionVelocityJointController::~SrhMixedPositionVelocityJointController()
  {
    sub_command_.shutdown();
  }

  bool SrhMixedPositionVelocityJointController::init(pr2_mechanism_model::RobotState *robot, const std::string &joint_name,
                                                     boost::shared_ptr<control_toolbox::Pid> pid_velocity)
  {
    ROS_DEBUG(" --------- ");
    ROS_DEBUG_STREAM("Init: " << joint_name);

    assert(robot);
    robot_ = robot;
    last_time_ = robot->getTime();


    //joint 0s
    if( joint_name.substr(3,1).compare("0") == 0)
    {
      has_j2 = true;
      std::string j1 = joint_name.substr(0,3) + "1";
      std::string j2 = joint_name.substr(0,3) + "2";
      ROS_DEBUG_STREAM("Joint 0: " << j1 << " " << j2);

      joint_state_ = robot_->getJointState(j1);
      if (!joint_state_)
      {
        ROS_ERROR("SrhMixedPositionVelocityController could not find joint named \"%s\"\n",
                  joint_name.c_str());
        return false;
      }

      joint_state_2 = robot_->getJointState(j2);
      if (!joint_state_2)
      {
        ROS_ERROR("SrhMixedPositionVelocityController could not find joint named \"%s\"\n",
                  joint_name.c_str());
        return false;
      }
      if (!joint_state_2->calibrated_)
      {
        ROS_ERROR("Joint %s not calibrated for SrhMixedPositionVelocityJointController", j2.c_str());
        return false;
      }
    }
    else //"normal" joints
    {
      has_j2 = false;

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
    }
    friction_compensator = boost::shared_ptr<sr_friction_compensation::SrFrictionCompensator>(new sr_friction_compensation::SrFrictionCompensator(joint_name));

    pid_controller_velocity_ = pid_velocity;

    serve_set_gains_ = node_.advertiseService("set_gains", &SrhMixedPositionVelocityJointController::setGains, this);

    ROS_DEBUG_STREAM(" joint_state name: " << joint_state_->joint_->name);
    ROS_DEBUG_STREAM(" In Init: " << getJointName() << " This: " << this
                     << " joint_state: "<<joint_state_ );

#ifdef DEBUG_PUBLISHER
    if( std::string("THJ2").compare(getJointName()) == 0)
    {
      ROS_INFO("Publishing debug infor for THJ2 mixed position/velocity controller");
      std::stringstream ss2;
      ss2 << getJointName() << "debug_velocity";
      debug_pub = n_tilde_.advertise<std_msgs::Float64>(ss2.str(), 2);
    }
#endif

    after_init();
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

    boost::shared_ptr<control_toolbox::Pid> pid_velocity = boost::shared_ptr<control_toolbox::Pid>( new control_toolbox::Pid() );;
    if (!pid_velocity->init(ros::NodeHandle(node_, "pid")))
      return false;


    controller_state_publisher_.reset(
      new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointControllerState>
      (node_, "state", 1));

    return init(robot, joint_name, pid_velocity);
  }


  void SrhMixedPositionVelocityJointController::starting()
  {
    if( has_j2 )
      command_ = joint_state_->position_ + joint_state_2->position_;
    command_ = joint_state_->position_;
    pid_controller_velocity_->reset();
    read_parameters();

    ROS_WARN("Reseting PID");
  }

  bool SrhMixedPositionVelocityJointController::setGains(sr_robot_msgs::SetMixedPositionVelocityPidGains::Request &req,
                                                         sr_robot_msgs::SetMixedPositionVelocityPidGains::Response &resp)
  {
    pid_controller_velocity_->setGains(req.p,req.i,req.d,req.i_clamp,-req.i_clamp);
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
    pid_controller_velocity_->getGains(p,i,d,i_max,i_min);
  }

  void SrhMixedPositionVelocityJointController::update()
  {
    if( !has_j2)
    {
      if (!joint_state_->calibrated_)
        return;
    }

    assert(robot_ != NULL);
    ros::Time time = robot_->getTime();
    assert(joint_state_->joint_);
    dt_= time - last_time_;

    if (!initialized_)
    {
      initialized_ = true;
      if( has_j2 )
        command_ = joint_state_->position_ + joint_state_2->position_;
      else
        command_ = joint_state_->position_;
    }

    //Compute velocity demand from position error:
    double error_position = 0.0;
    if( has_j2 )
    {
      error_position = (joint_state_->position_ + joint_state_2->position_) - command_;
      ROS_DEBUG_STREAM("j0: " << joint_state_->position_ + joint_state_2->position_);
    }
    else
      error_position = joint_state_->position_ - command_;

    double commanded_velocity = 0.0;
    double error_velocity = 0.0;
    double commanded_effort = 0.0;

    //don't compute the error if we're in the deadband.
    if( !hysteresis_deadband.is_in_deadband(command_, error_position, position_deadband) )
    {
      //compute the velocity demand from the simple interpoler
      commanded_velocity = compute_velocity_demand(error_position);

      //velocity loop:
      if( has_j2 )
        error_velocity = (joint_state_->velocity_ + joint_state_->velocity_) / 2.0 - commanded_velocity;
      else
        error_velocity = joint_state_->velocity_ - commanded_velocity;
      commanded_effort = pid_controller_velocity_->updatePid(error_velocity, dt_);

      //clamp the result to max force
      commanded_effort = min( commanded_effort, max_force_demand );
      commanded_effort = max( commanded_effort, -max_force_demand );

      //Friction compensation
      //if( std::string("FFJ3").compare( getJointName() ) == 0 )
      //  ROS_INFO_STREAM(getJointName() << ": before fc: velocity demand=" << commanded_velocity << " force demand=" << commanded_effort << " / error: " << error_velocity );
      if( has_j2 )
        commanded_effort += friction_compensator->friction_compensation( joint_state_->position_ + joint_state_2->position_ , int(commanded_effort), friction_deadband );
      else
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
    if( has_j2 )
      joint_state_2->commanded_effort_ = commanded_effort;
    else
      joint_state_->commanded_effort_ = commanded_effort;
    if(loop_count_ % 10 == 0)
    {
#ifdef DEBUG_PUBLISHER
      if( std::string("THJ2").compare(getJointName()) == 0)
      {
        std_msgs::Float64 msg;
        msg.data = commanded_velocity;
        debug_pub.publish(msg);
      }
#endif

      if(controller_state_publisher_ && controller_state_publisher_->trylock())
      {
        controller_state_publisher_->msg_.header.stamp = time;
        controller_state_publisher_->msg_.set_point = command_;
        if( has_j2 )
        {
          controller_state_publisher_->msg_.process_value = (joint_state_->position_ + joint_state_2->position_);
          controller_state_publisher_->msg_.process_value_dot = (joint_state_->velocity_ + joint_state_2->velocity_) / 2.0;
        }
        else
        {
          controller_state_publisher_->msg_.process_value = joint_state_->position_;
          controller_state_publisher_->msg_.process_value_dot = joint_state_->velocity_;
        }

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

  void SrhMixedPositionVelocityJointController::read_parameters()
  {
    node_.param<double>("pid/max_force", max_force_demand, 1023.0);
    node_.param<double>("pid/min_velocity", min_velocity_, -1.0);
    node_.param<double>("pid/max_velocity", max_velocity_, 1.0);
    node_.param<double>("pid/velocity_slope", slope_velocity_, 10.0);
    node_.param<double>("pid/position_deadband", position_deadband, 0.015);
    node_.param<int>("pid/friction_deadband", friction_deadband, 5);

    set_min_max_position_errors_();
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/



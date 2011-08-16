/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "sr_edc_mechanism_controllers/srh_position_controller.h"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.h"
#include <math.h>

PLUGINLIB_DECLARE_CLASS(sr_edc_mechanism_controllers, SrhPositionController, controller::SrhPositionController, pr2_controller_interface::Controller)

using namespace std;

namespace controller {

  SrhPositionController::SrhPositionController()
    : joint_state_(NULL), command_(0),
      loop_count_(0),  initialized_(false), robot_(NULL), last_time_(0),
      max_force_demand(1000.)
  {
  }

  SrhPositionController::~SrhPositionController()
  {
    sub_command_.shutdown();
  }

  bool SrhPositionController::init(pr2_mechanism_model::RobotState *robot, const std::string &joint_name,
                                   const control_toolbox::Pid &pid)
  {
    ROS_DEBUG(" --------- ");
    ROS_DEBUG_STREAM("Init: " << joint_name);

    assert(robot);
    robot_ = robot;
    last_time_ = robot->getTime();

    joint_state_ = robot_->getJointState(joint_name);
    if (!joint_state_)
    {
      ROS_ERROR("SrhPositionController could not find joint named \"%s\"\n",
                joint_name.c_str());
      return false;
    }
    if (!joint_state_->calibrated_)
    {
      ROS_ERROR("Joint %s not calibrated for SrhPositionController", joint_name.c_str());
      return false;
    }

    friction_interpoler = boost::shared_ptr<shadow_robot::JointCalibration>( new shadow_robot::JointCalibration( read_friction_map() ) );

    pid_controller_ = pid;

    serve_set_gains_ = node_.advertiseService("set_gains", &SrhPositionController::setGains, this);

    ROS_DEBUG_STREAM(" joint_state name: " << joint_state_->joint_->name);
    ROS_DEBUG_STREAM(" In Init: " << getJointName() << " This: " << this
                     << " joint_state: "<<joint_state_ );

    return true;
  }

  bool SrhPositionController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
  {
    assert(robot);
    node_ = n;

    std::string joint_name;
    if (!node_.getParam("joint", joint_name)) {
      ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }

    control_toolbox::Pid pid;
    if (!pid.init(ros::NodeHandle(node_, "pid")))
      return false;

    controller_state_publisher_.reset(
      new realtime_tools::RealtimePublisher<pr2_controllers_msgs::JointControllerState>
      (node_, "state", 1));

    sub_command_ = node_.subscribe<std_msgs::Float64>("command", 1, &SrhPositionController::setCommandCB, this);

    return init(robot, joint_name, pid);
  }


  void SrhPositionController::starting()
  {
    command_ = joint_state_->position_;
    pid_controller_.reset();
    ROS_WARN("Reseting PID");
  }

  bool SrhPositionController::setGains(sr_robot_msgs::SetPidGains::Request &req,
                                       sr_robot_msgs::SetPidGains::Response &resp)
  {
    pid_controller_.setGains(req.p,req.i,req.d,req.i_clamp,-req.i_clamp);
    max_force_demand = req.max_force;

    return true;
  }

  void SrhPositionController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, double &max_force)
  {
    pid_controller_.getGains(p,i,d,i_max,i_min);
    max_force = max_force_demand;
  }

  std::string SrhPositionController::getJointName()
  {
    ROS_DEBUG_STREAM(" joint_state: "<<joint_state_ << " This: " << this);

    return joint_state_->joint_->name;
  }

// Set the joint position command
  void SrhPositionController::setCommand(double cmd)
  {
    command_ = cmd;
  }

// Return the current position command
  void SrhPositionController::getCommand(double & cmd)
  {
    cmd = command_;
  }

  void SrhPositionController::update()
  {
    if (!joint_state_->calibrated_)
      return;

    assert(robot_ != NULL);
    double error(0);
    ros::Time time = robot_->getTime();
    assert(joint_state_->joint_);
    dt_= time - last_time_;

    if (!initialized_)
    {
      initialized_ = true;
      command_ = joint_state_->position_;
    }

    error = joint_state_->position_ - command_;

    double commanded_effort = pid_controller_.updatePid(error, joint_state_->velocity_, dt_);

    //if( std::string("FFJ3").compare(getJointName()) == 0)
    //  ROS_INFO_STREAM(getJointName() << ": before fc: effort=" << commanded_effort << " / pos: " << joint_state_->position_  << " / max force: " << max_force_demand);
    commanded_effort += friction_compensation( joint_state_->position_ );

    ROS_DEBUG_STREAM(getJointName() << ": after fc: effort=" << commanded_effort );

    commanded_effort = min( commanded_effort, max_force_demand );
    commanded_effort = max( commanded_effort, -max_force_demand );

    joint_state_->commanded_effort_ = commanded_effort;

    if(loop_count_ % 10 == 0)
    {
      if(controller_state_publisher_ && controller_state_publisher_->trylock())
      {
        controller_state_publisher_->msg_.header.stamp = time;
        controller_state_publisher_->msg_.set_point = command_;
        controller_state_publisher_->msg_.process_value = joint_state_->position_;
        controller_state_publisher_->msg_.process_value_dot = joint_state_->velocity_;
        controller_state_publisher_->msg_.error = error;
        controller_state_publisher_->msg_.time_step = dt_.toSec();
        controller_state_publisher_->msg_.command = commanded_effort;

        double max_force, imin = 0.0;
        getGains(controller_state_publisher_->msg_.p,
                 controller_state_publisher_->msg_.i,
                 controller_state_publisher_->msg_.d,
                 imin,
                 controller_state_publisher_->msg_.i_clamp,
                 max_force);
        controller_state_publisher_->unlockAndPublish();
      }
    }
    loop_count_++;

    last_time_ = time;
  }

  double SrhPositionController::friction_compensation( double position )
  {
    double compensation = 0.0;
    compensation = friction_interpoler->compute( position );
    return compensation;
  }

  void SrhPositionController::setCommandCB(const std_msgs::Float64ConstPtr& msg)
  {
    command_ = msg->data;
  }


  std::vector<joint_calibration::Point> SrhPositionController::read_friction_map()
  {
    std::vector<joint_calibration::Point> friction_map;
    std::string param_name = "/sr_friction_map";

    bool joint_not_found = true;

    XmlRpc::XmlRpcValue calib;
    node_.getParam(param_name, calib);

    ROS_DEBUG_STREAM("  Reading friction for: " <<  getJointName());
    ROS_DEBUG_STREAM(" value: " << calib);

    ROS_ASSERT(calib.getType() == XmlRpc::XmlRpcValue::TypeArray);
    //iterate on all the joints
    for(int32_t index_cal = 0; index_cal < calib.size(); ++index_cal)
    {
      //check the calibration is well formatted:
      // first joint name, then calibration table
      ROS_ASSERT(calib[index_cal][0].getType() == XmlRpc::XmlRpcValue::TypeString);
      ROS_ASSERT(calib[index_cal][1].getType() == XmlRpc::XmlRpcValue::TypeArray);

      std::string joint_name = static_cast<std::string> (calib[index_cal][0]);

      ROS_DEBUG_STREAM("  Checking joint name: "<< joint_name << " / " << getJointName());
      if(  joint_name.compare( getJointName() ) != 0 )
        continue;

      ROS_DEBUG_STREAM("   OK: joint name = "<< joint_name);

      joint_not_found = false;
      //now iterates on the calibration table for the current joint
      for(int32_t index_table=0; index_table < calib[index_cal][1].size(); ++index_table)
      {
        ROS_ASSERT(calib[index_cal][1][index_table].getType() == XmlRpc::XmlRpcValue::TypeArray);
        //only 2 values per calibration point: raw and calibrated (doubles)
        ROS_ASSERT(calib[index_cal][1][index_table].size() == 2);
        ROS_ASSERT(calib[index_cal][1][index_table][0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        ROS_ASSERT(calib[index_cal][1][index_table][1].getType() == XmlRpc::XmlRpcValue::TypeDouble);


        joint_calibration::Point point_tmp;
        point_tmp.raw_value = static_cast<double> (calib[index_cal][1][index_table][0]);
        point_tmp.calibrated_value = static_cast<double> (calib[index_cal][1][index_table][1]);
        friction_map.push_back(point_tmp);
      }

      break;
    }

    if( joint_not_found )
    {
      ROS_WARN_STREAM("  No friction compensation for: " << getJointName() );

      joint_calibration::Point point_tmp;
      point_tmp.raw_value = 0.0;
      point_tmp.calibrated_value = 0.0;
      friction_map.push_back(point_tmp);
      point_tmp.raw_value = 1.0;
      friction_map.push_back(point_tmp);
    }

    ROS_DEBUG_STREAM(" Friction map[" << getJointName() << "]");
    for( unsigned int i=0; i<friction_map.size(); ++i )
      ROS_DEBUG_STREAM("    -> position=" << friction_map[i].raw_value << " compensation: " << friction_map[i].calibrated_value);

    return friction_map;
  } //end read_friction_map


}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/



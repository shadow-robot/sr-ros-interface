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

#include "sr_edc_mechanism_controllers/srh_fake_joint_calibration_controller.h"
#include "ros/time.h"
#include "pluginlib/class_list_macros.h"
#include <boost/algorithm/string.hpp>
#include <string>
#include <sr_robot_msgs/ForceController.h>

PLUGINLIB_DECLARE_CLASS(sr_edc_mechanism_controllers, SrhFakeJointCalibrationController, controller::SrhFakeJointCalibrationController, pr2_controller_interface::Controller)

using namespace std;

namespace controller {

  SrhFakeJointCalibrationController::SrhFakeJointCalibrationController()
    : robot_(NULL), last_publish_time_(0), state_(INITIALIZED),
      actuator_(NULL), joint_(NULL), transmission_(NULL)
  {
  }

  SrhFakeJointCalibrationController::~SrhFakeJointCalibrationController()
  {
  }

  bool SrhFakeJointCalibrationController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
  {
    robot_ = robot;
    node_ = n;
    // Joint

    std::string joint_name;
    if (!node_.getParam("joint", joint_name))
    {
      ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }
    if (!(joint_ = robot->getJointState(joint_name)))
    {
      ROS_ERROR("Could not find joint %s (namespace: %s)",
                joint_name.c_str(), node_.getNamespace().c_str());
      return false;
    }
    joint_name_ = joint_name;

    // Actuator
    std::string actuator_name;
    if (!node_.getParam("actuator", actuator_name))
    {
      ROS_ERROR("No actuator given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }
    if (!(actuator_ = robot->model_->getActuator(actuator_name)))
    {
      ROS_ERROR("Could not find actuator %s (namespace: %s)",
                actuator_name.c_str(), node_.getNamespace().c_str());
      return false;
    }
    actuator_name_ = actuator_name;

    // Transmission
    std::string transmission_name;
    if (!node_.getParam("transmission", transmission_name))
    {
      ROS_ERROR("No transmission given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }
    if (!(transmission_ = robot->model_->getTransmission(transmission_name)))
    {
      ROS_ERROR("Could not find transmission %s (namespace: %s)",
                transmission_name.c_str(), node_.getNamespace().c_str());
      return false;
    }

    // "Calibrated" topic
    pub_calibrated_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Empty>(node_, "calibrated", 1));

    return true;
  }


  void SrhFakeJointCalibrationController::update()
  {
    assert(joint_);
    assert(actuator_);

    switch(state_)
    {
    case INITIALIZED:
      state_ = BEGINNING;
      break;
    case BEGINNING:
      initialize_pids();
      joint_->calibrated_ = true;
      state_ = CALIBRATED;
      break;
    case CALIBRATED:
      if (pub_calibrated_)
      {
        if (last_publish_time_ + ros::Duration(0.5) < robot_->getTime())
        {
          assert(pub_calibrated_);
          if (pub_calibrated_->trylock())
          {
            last_publish_time_ = robot_->getTime();
            pub_calibrated_->unlockAndPublish();
          }
        }
      }
      break;
    }
  }

  void SrhFakeJointCalibrationController::initialize_pids()
  {
    //read the parameters from the parameter server and set the pid
    // values.
    std::stringstream full_param;

    int f, p, i, d, imax, max_pwm, sg_left, sg_right, deadband, sign;
    std::string act_name = boost::to_lower_copy(actuator_name_);

    full_param << "/" << act_name << "/pid/f";
    node_.param<int>(full_param.str(), f, 0);
    full_param.str("");
    full_param << "/" << act_name << "/pid/p";
    node_.param<int>(full_param.str(), p, 0);
    full_param.str("");
    full_param << "/" << act_name << "/pid/i";
    node_.param<int>(full_param.str(), i, 0);
    full_param.str("");
    full_param << "/" << act_name << "/pid/d";
    node_.param<int>(full_param.str(), d, 0);
    full_param.str("");
    full_param << "/" << act_name << "/pid/imax";
    node_.param<int>(full_param.str(), imax, 0);
    full_param.str("");
    full_param << "/" << act_name << "/pid/max_pwm";
    ROS_ERROR_STREAM("Full param: "<< full_param.str());
    node_.param<int>(full_param.str(), max_pwm, 0);
    full_param.str("");
    full_param << "/" << act_name << "/pid/sg_left";
    node_.param<int>(full_param.str(), sg_left, 0);
    full_param.str("");
    full_param << "/" << act_name << "/pid/sg_right";
    node_.param<int>(full_param.str(), sg_right, 0);
    full_param.str("");
    full_param << "/" << act_name << "/pid/deadband";
    node_.param<int>(full_param.str(), deadband, 0);
    full_param.str("");
    full_param << "/" << act_name << "/pid/sign";
    node_.param<int>(full_param.str(), sign, 0);
    full_param.str("");

    std::string act_name_upper = boost::to_upper_copy( act_name );
    std::string service_name = "/realtime_loop/change_force_PID_" + act_name_upper;
    if( ros::service::waitForService (service_name, ros::Duration(2.0)) )
    {
      sr_robot_msgs::ForceController::Request pid_request;
      pid_request.maxpwm = max_pwm;
      pid_request.sgleftref = sg_left;
      pid_request.sgrightref = sg_right;
      pid_request.f = f;
      pid_request.p = p;
      pid_request.i = i;
      pid_request.d = d;
      pid_request.imax = imax;
      pid_request.deadband = deadband;
      pid_request.sign = sign;
      sr_robot_msgs::ForceController::Response pid_response;
      if( ros::service::call(service_name, pid_request, pid_response) )
      {
        return;
      }
    }

    ROS_WARN_STREAM( "Didn't load the force pid settings for the motor in joint " << act_name );
  }

} // namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

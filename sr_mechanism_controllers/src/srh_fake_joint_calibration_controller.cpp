/**
 * @file   srh_fake_joint_calibration_controller.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Aug 23 12:03:37 2011
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
 * @brief  A Fake joint calibration controller. Only loads the force pid settings
 * from the parameter server.
 *
 *
 */

#include "sr_mechanism_controllers/srh_fake_joint_calibration_controller.h"
#include "ros/time.h"
#include "pluginlib/class_list_macros.h"
#include <boost/algorithm/string.hpp>
#include <string>
#include <sr_robot_msgs/ForceController.h>

PLUGINLIB_DECLARE_CLASS(sr_mechanism_controllers, SrhFakeJointCalibrationController, controller::SrhFakeJointCalibrationController, pr2_controller_interface::Controller)

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

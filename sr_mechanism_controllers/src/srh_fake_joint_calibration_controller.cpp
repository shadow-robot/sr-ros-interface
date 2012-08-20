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
#include <std_srvs/Empty.h>

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
    ///Reset the motor to make sure we have the proper 0 + correct PID settings
    std::string service_name = "/realtime_loop/reset_motor_" + boost::to_upper_copy(actuator_name_);
    if( ros::service::waitForService (service_name, ros::Duration(2.0)) )
    {
      ros::ServiceClient client = node_.serviceClient<std_srvs::Empty>(service_name);
      std_srvs::Empty srv;
      if( client.call(srv) )
      {
        return;
      }
    }
  }

} // namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

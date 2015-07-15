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

PLUGINLIB_EXPORT_CLASS( controller::SrhFakeJointCalibrationController, controller_interface::ControllerBase)

using namespace std;

namespace controller {

  SrhFakeJointCalibrationController::SrhFakeJointCalibrationController()
    : robot_(NULL),
      last_publish_time_(0),
      calibration_state_(IS_INITIALIZED),
      actuator_(NULL),
      joint_(NULL)
  {
  }

  bool SrhFakeJointCalibrationController::init(ros_ethercat_model::RobotState *robot, ros::NodeHandle &n)
  {
    ROS_ASSERT(robot);
    robot_ = robot;
    node_ = n;

    // robot_id robot_id_, joint_prefix_, ns_
    if (node_.getParam("robot_id", robot_id_)
        && (!robot_id_.empty()))
    {
      joint_prefix_ = robot_id_ + "_";
      ns_ = robot_id_ + "/";
    }


    // Joint
    if (!node_.getParam("joint", joint_name_))
    {
      ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }
    if (!(joint_ = robot->getJointState(joint_prefix_ + joint_name_)))
    {
      ROS_ERROR("Could not find joint %s (namespace: %s)",
                (joint_prefix_ + joint_name_).c_str(), node_.getNamespace().c_str());
      return false;
    }

    // Actuator
    if (!node_.getParam("actuator", actuator_name_))
    {
      ROS_ERROR("No actuator given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }
    if (!(actuator_ = robot->getActuator(joint_prefix_ + actuator_name_)))
    {
      ROS_ERROR("Could not find actuator %s (namespace: %s)",
                (joint_prefix_ + actuator_name_).c_str(), node_.getNamespace().c_str());
      return false;
    }

    // Transmission
    string transmission_name;
    if (!node_.getParam("transmission", transmission_name))
    {
      ROS_ERROR("No transmission given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }

    // "Calibrated" topic
    pub_calibrated_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Empty>(node_, "calibrated", 1));

    return true;
  }


  void SrhFakeJointCalibrationController::update(const ros::Time& time, const ros::Duration& period)
  {
    ROS_ASSERT(joint_);
    ROS_ASSERT(actuator_);

    switch(calibration_state_)
    {
    case IS_INITIALIZED:
      calibration_state_ = BEGINNING;
      break;
    case BEGINNING:
      initialize_pids();
      joint_->calibrated_ = true;
      calibration_state_ = CALIBRATED;
      //We add the following line to delay for some time the first publish and allow the correct initialization of the subscribers in calibrate.py
      last_publish_time_ = robot_->getTime();
      break;
    case CALIBRATED:
      if (pub_calibrated_)
      {
        if (last_publish_time_ + ros::Duration(0.5) < robot_->getTime())
        {
          ROS_ASSERT(pub_calibrated_);
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
    // trim any prefix in the actuator_name for low level driver to find it
    std::string lowlevel_actuator_name= actuator_name_.substr(actuator_name_.size()-4,4);
    string service_name = "realtime_loop/" + ns_ + "reset_motor_" + boost::to_upper_copy(lowlevel_actuator_name);
    if( ros::service::waitForService (service_name, ros::Duration(2.0)) )
    {
      std_srvs::Empty srv;
      if (ros::service::call(service_name, srv))
      {
        return;
      }
      else
      {
        ROS_WARN("Reset failed: %s", service_name.c_str());
      }
    }
  }

} // namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

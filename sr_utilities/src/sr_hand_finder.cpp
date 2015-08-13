/*
* File:  sf_hand_finder.cpp
* Author: Vahid Aminzadeh <vahid@shadowrobot.com>
* Copyright 2015 Shadow Robot Company Ltd.
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
* @brief see README.md
*/

#include "sr_utilities/sr_hand_finder.hpp"
#include "ros/ros.h"
#include <vector>
#include <string>
#include <iostream>
#include <map>
#include <ros/package.h>

using std::string;
using std::map;
using std::vector;
namespace shadow_robot
{
const size_t SrHandFinder::number_of_joints_ = 20;
const char* SrHandFinder::joint_names_[] = {"FFJ1", "FFJ2", "FFJ3", "FFJ4", "MFJ1", "MFJ2", "MFJ3", "MFJ4",
                                            "RFJ1", "RFJ2", "RFJ3", "RFJ4", "LFJ1", "LFJ2", "LFJ3", "LFJ4",
                                            "LFJ5", "THJ1", "THJ2", "THJ3", "THJ4", "THJ5", "WRJ1", "WRJ2"};
SrHandFinder::SrHandFinder()
{
  map<string, string> mapping_map;
  ros::param::get("hand/mapping", hand_config_.mapping_);
  ros::param::get("hand/joint_prefix", hand_config_.joint_prefix_);

  for (map<string, string>::const_iterator iter = hand_config_.mapping_.begin();
      iter != hand_config_.mapping_.end(); ++iter)
  {
    ROS_INFO_STREAM("detected hands are \n" << "hand serial:" << iter->first << " hand_id:" << iter->second);
  }
  generate_joints_with_prefix();
  generate_calibration_path();
  generate_hand_controller_tuning_path();
}
void SrHandFinder::generate_joints_with_prefix()
{
  for (map<string, string>::const_iterator prefix_iter = hand_config_.joint_prefix_.begin();
      prefix_iter != hand_config_.joint_prefix_.end(); ++prefix_iter)
  {
    joints_[prefix_iter->second].resize(number_of_joints_);
    for (size_t joint_counter = 0; joint_counter != number_of_joints_; ++joint_counter)
    {
      joints_[prefix_iter->second][joint_counter] = prefix_iter->second + joint_names_[joint_counter];
    }
  }
}
void SrHandFinder::generate_calibration_path()
{
  string ethercat_path = ros::package::getPath("sr_ethercat_hand_config");
  for (map<string, string>::const_iterator mapping_iter = hand_config_.mapping_.begin();
      mapping_iter != hand_config_.mapping_.end(); ++mapping_iter)
  {
    calibration_path_[mapping_iter->second] = ethercat_path + "/calibrations/"
        + mapping_iter->second + "/" + "calibration.yaml";
  }
}
void SrHandFinder::generate_hand_controller_tuning_path()
{
  string ethercat_path = ros::package::getPath("sr_ethercat_hand_config");
  for (map<string, string>::const_iterator mapping_iter = hand_config_.mapping_.begin();
        mapping_iter != hand_config_.mapping_.end(); ++mapping_iter)
  {
    hand_controller_tuning_.friction_compensation_[mapping_iter->second] =
        ethercat_path + "/controls/" + "friction_compensation.yaml";
    hand_controller_tuning_.motor_control_[mapping_iter->second] =
        ethercat_path + "/controls/motors/" + mapping_iter->second + "/motor_board_effort_controllers.yaml";
    string host_path(ethercat_path + "/controls/host/" + mapping_iter->second + "/");
    hand_controller_tuning_.host_control_[mapping_iter->second].push_back(
        host_path + "sr_edc_calibration_controllers.yaml");
    hand_controller_tuning_.host_control_[mapping_iter->second].push_back(
        host_path + "sr_edc_joint_velocity_controllers_PWM.yaml");
    hand_controller_tuning_.host_control_[mapping_iter->second].push_back(
        host_path + "sr_edc_effort_controllers_PWM.yaml");
    hand_controller_tuning_.host_control_[mapping_iter->second].push_back(
        host_path + "sr_edc_joint_velocity_controllers.yaml");
    hand_controller_tuning_.host_control_[mapping_iter->second].push_back(
        host_path + "sr_edc_effort_controllers.yaml");

    hand_controller_tuning_.host_control_[mapping_iter->second].push_back(
        host_path + "sr_edc_mixed_position_velocity_joint_controllers_PWM.yaml");
    hand_controller_tuning_.host_control_[mapping_iter->second].push_back(
        host_path + "sr_edc_joint_position_controllers_PWM.yaml");
    hand_controller_tuning_.host_control_[mapping_iter->second].push_back(
        host_path + "sr_edc_mixed_position_velocity_joint_controllers.yaml");
    hand_controller_tuning_.host_control_[mapping_iter->second].push_back(
        host_path + "sr_edc_joint_position_controllers.yaml");
  }
}
map<string, vector<string> > SrHandFinder::get_joints()
{
  return joints_;
}
map<string, string> SrHandFinder::get_calibration_path()
{
  return calibration_path_;
}
HandControllerTuning SrHandFinder::get_hand_controller_tuning()
{
  return hand_controller_tuning_;
}
} /* namespace shadow_robot */

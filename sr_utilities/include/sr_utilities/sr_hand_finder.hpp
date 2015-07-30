/*
* File:  sf_hand_finder.hpp
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

#pragma once
#include "ros/ros.h"
#include <map>
#include <string>
#include <vector>
namespace shadow_robot
{

struct HandConfig
{
  std::map<std::string, std::string> mapping_;
  std::map<std::string, std::string> joint_prefix_;
};

struct HandControllerTuning
{
  std::map<std::string, std::string> friction_compensation_;
  std::map<std::string, std::vector<std::string> > host_control_;
  std::map<std::string, std::string> motor_control_;
};

class SrHandFinder
{
public:
  std::map<std::string, std::vector<std::string> > get_joints();
  std::map<std::string, std::string> get_calibration_path();
  HandControllerTuning get_hand_controller_tuning();
private:
  static const size_t number_of_joints_;
  static const char* joint_names_[];
  ros::NodeHandle node_handle_;
  HandConfig hand_config_;
  HandControllerTuning hand_controller_tuning_;
  std::map<std::string, std::vector<std::string> > joints_;
  std::map<std::string, std::string> calibration_path_;
  void generate_joints_with_prefix();
  void generate_calibration_path();
  void generate_hand_controller_tuning_path();



public:
  SrHandFinder();
  virtual ~SrHandFinder(){}
};

} /* namespace shadow_robot */

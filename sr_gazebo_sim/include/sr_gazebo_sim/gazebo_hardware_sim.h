/**
* @file   gazebo_hardware_sim.h
* @author Andriy Petlovanyy <software@shadowrobot.com>
*
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
*
* @brief Gazebo custom hardware implementation.
*
*/

#ifndef __SR_GAZEBO_SIM_GAZEBO_HARDWARE_SIM_H
#define __SR_GAZEBO_SIM_GAZEBO_HARDWARE_SIM_H

#include <string>
#include <vector>
#include <boost/unordered_map.hpp>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include "ros_ethercat_model/robot_state.hpp"
#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <urdf/model.h>

namespace sr_gazebo_sim
{

class SrGazeboHWSim : public gazebo_ros_control::DefaultRobotHWSim
{
public:
  SrGazeboHWSim();

  bool initSim(
      const std::string &robot_namespace,
      ros::NodeHandle model_nh,
      gazebo::physics::ModelPtr parent_model,
      const urdf::Model *const urdf_model,
      std::vector <transmission_interface::TransmissionInfo> transmissions);

  void readSim(ros::Time time, ros::Duration period);

  void writeSim(ros::Time time, ros::Duration period);

protected:
  template <class T>
  void fixJointName(std::vector<T> *items, const std::string old_joint_name, const std::string new_joint_name) const;

  void addFakeTransmissionsForJ0(std::vector<transmission_interface::TransmissionInfo> *transmissions);

  void initializeFakeRobotState(const urdf::Model*const urdf_model,
                                const std::vector<transmission_interface::TransmissionInfo> &transmissions);

  bool isHandJoint(const std::vector<transmission_interface::TransmissionInfo> &transmissions,
                     const std::string &joint_name) const;

  void registerSecondHardwareInterface(std::vector<transmission_interface::TransmissionInfo> transmissions);

  static const std::string j0_transmission_name;
  static const std::string simple_transmission_name;

  ros_ethercat_model::RobotState fake_state_;
  boost::unordered_map<std::string, std::string> j2_j1_joints_;
};

typedef boost::shared_ptr <SrGazeboHWSim> SrGazeboHWSimPtr;

}  // namespace sr_gazebo_sim


#endif  // __SR_GAZEBO_SIM_GAZEBO_HARDWARE_SIM_H

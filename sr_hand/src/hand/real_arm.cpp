/**
 * @file   real_arm.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Tue Jun 29 14:56:10 2010
 *
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
 * @brief The real arm can be used as a simulator. It modelizes the Shadow Robot muscle arm.
 *
 */

#include "sr_hand/hand/real_arm.h"
//our robot code
#include <robot/robot.h>
#include <robot/hand.h>
#include <robot/hand_protocol.h>

#include <time.h>
#include <ros/ros.h>

namespace shadowrobot
{
RealArm::RealArm() :
  SRArticulatedRobot()
{

  /* We need to attach the program to the robot, or fail if we cannot. */
  if (robot_init() < 0)
  {
    ROS_FATAL("Robot interface broken\n");
    ROS_BREAK();
  }

  /* We need to attach the program to the hand as well, or fail if we cannot. */
  if (hand_init() < 0)
  {
    ROS_FATAL("Arm interface broken\n");
    ROS_BREAK();
  }

  srand(time(NULL));
  initializeMap();
}

RealArm::~RealArm()
{
}

void RealArm::initializeMap()
{
  joints_map_mutex.lock();
  JointData tmpData;

  tmpData.position = 0.0;
  tmpData.target = 0.0;
  tmpData.temperature = 0.0;
  tmpData.current = 0.0;
  tmpData.force = 0.0;
  tmpData.flags = "";

  for (unsigned int i = START_OF_ARM; i < NUM_HAND_JOINTS; ++i)
  {
    std::string name = hand_joints[i].joint_name;
    tmpData.jointIndex = i;

    joints_map[name] = tmpData;

    ROS_INFO("NAME[%d]: %s ", i, name.c_str());
  }

  joints_map_mutex.unlock();
}

short RealArm::sendupdate(std::string joint_name, double target)
{
  joints_map_mutex.lock();

  //search the sensor in the hash_map
  JointsMap::iterator iter = joints_map.find(joint_name);

  if (iter != joints_map.end())
  {
    JointData tmpData = joints_map.find(joint_name)->second;
    int index_arm_joints = tmpData.jointIndex;

    //trim to the correct range
    if (target < hand_joints[index_arm_joints].min_angle)
      target = hand_joints[index_arm_joints].min_angle;
    if (target > hand_joints[index_arm_joints].max_angle)
      target = hand_joints[index_arm_joints].max_angle;

    //here we update the actual hand angles
    robot_update_sensor(&hand_joints[index_arm_joints].joint_target, target);
    joints_map_mutex.unlock();
    return 0;
  }

  ROS_DEBUG("Joint %s not found", joint_name.c_str());

  joints_map_mutex.unlock();
  return -1;
}

JointData RealArm::getJointData(std::string joint_name)
{
  joints_map_mutex.lock();
  JointsMap::iterator iter = joints_map.find(joint_name);

  //joint not found
  if (iter == joints_map.end())
  {
    ROS_ERROR("Joint %s not found.", joint_name.c_str());
    JointData noData;
    noData.position = 0.0;
    noData.target = 0.0;
    noData.temperature = 0.0;
    noData.current = 0.0;
    noData.force = 0.0;
    noData.flags = "";
    noData.jointIndex = 0;

    joints_map_mutex.unlock();
    return noData;
  }

  //joint found
  JointData tmpData = iter->second;
  int index = tmpData.jointIndex;

  tmpData.position = robot_read_sensor(&hand_joints[index].position);
  tmpData.target = robot_read_sensor(&hand_joints[index].joint_target);

  joints_map[joint_name] = tmpData;

  joints_map_mutex.unlock();
  return tmpData;
}

SRArticulatedRobot::JointsMap RealArm::getAllJointsData()
{
  //update the map for each joints
  for (JointsMap::const_iterator it = joints_map.begin(); it != joints_map.end(); ++it)
    getJointData(it->first);

  JointsMap tmp = JointsMap(joints_map);

  //return the map
  return tmp;
}

short RealArm::setContrl(std::string contrlr_name, JointControllerData ctrlr_data)
{
  ROS_WARN("The setContrl method is not yet implemented");
  return 0;
}

JointControllerData RealArm::getContrl(std::string contrlr_name)
{
  ROS_WARN("The getContrl method is not yet implemented");
  JointControllerData no_result;
  return no_result;
}

short RealArm::setConfig(std::vector<std::string> myConfig)
{
  ROS_WARN("The set config function is not implemented in the virtual arm.");
  return 0;
}

void RealArm::getConfig(std::string joint_name)
{
  ROS_WARN("The get config function is not implemented in the virtual arm.");
}

std::vector<DiagnosticData> RealArm::getDiagnostics()
{
  joints_map_mutex.lock();
  std::vector<DiagnosticData> returnVect;

  //TODO: read diagnostic data from the robot

  for (JointsMap::const_iterator it = joints_map.begin(); it != joints_map.end(); ++it)
  {
    DiagnosticData tmpDiag;
    tmpDiag.joint_name = it->first;
    tmpDiag.level = 0;
    tmpDiag.flags = "";
    tmpDiag.target_sensor_num = 0;
    tmpDiag.target = it->second.target;
    tmpDiag.position_sensor_num = 0;
    tmpDiag.position = it-> second.position;

    returnVect.push_back(tmpDiag);
  }

  joints_map_mutex.unlock();
  return returnVect;
}

} //end namespace

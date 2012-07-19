/**
 * @file   CAN_compatibility_arm.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @author Guillaume Walck (UPMC)
 * @date   Sun Nov 20 2011
 *
* Copyright 2011 UPMC
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
 * @brief
 *
 *
 */

#include "sr_hand/hand/CAN_compatibility_arm.hpp"
#include <sr_utilities/sr_math_utils.hpp>

#include <time.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace shadowrobot
{
  ROS_DEPRECATED CANCompatibilityArm::CANCompatibilityArm() :
    SRArticulatedRobot(), n_tilde("~")
  {
    ROS_WARN("This interface is deprecated, you should probably use the interface provided by the CAN driver directly.");

    initializeMap();

    joint_state_subscriber = node.subscribe("joint_states", 2, &CANCompatibilityArm::joint_states_callback, this);

  }

  CANCompatibilityArm::~CANCompatibilityArm()
  {
  }

  void CANCompatibilityArm::initializeMap()
  {
    joints_map_mutex.lock();
    JointData tmpData;

    std::string controller_suffix;
    std::string searched_param;
    n_tilde.searchParam("controller_suffix", searched_param);
    n_tilde.param(searched_param, controller_suffix, std::string("position_controller"));
    std::string topic_prefix = "/sa_";
    std::string topic_suffix = "_"+controller_suffix+"/command";
    std::string full_topic = "";

    tmpData.min = -45.0;
    tmpData.max = 45.0;

    full_topic = topic_prefix + "sr" + topic_suffix;
    CAN_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    int tmp_index = 0;
    tmpData.publisher_index = tmp_index;
    joints_map["ShoulderJRotate"] = tmpData;

    tmpData.min = 0.0;
    tmpData.max = 80.0;

    full_topic = topic_prefix + "ss" + topic_suffix;
    CAN_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["ShoulderJSwing"] = tmpData;

    tmpData.min = 20.0;
    tmpData.max = 120.0;
    full_topic = topic_prefix + "es" + topic_suffix;
    CAN_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["ElbowJSwing"] = tmpData;

    tmpData.min = -80.0;
    tmpData.max = 80.0;

    full_topic = topic_prefix + "er" + topic_suffix;
    CAN_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["ElbowJRotate"] = tmpData;

    joints_map_mutex.unlock();
  }

  short CANCompatibilityArm::sendupdate( std::string joint_name, double target )
  {
    if( !joints_map_mutex.try_lock() )
      return -1;
    JointsMap::iterator iter = joints_map.find(joint_name);
    std_msgs::Float64 target_msg;

    //not found
    if( iter == joints_map.end() )
    {
      ROS_DEBUG("Joint %s not found", joint_name.c_str());

      joints_map_mutex.unlock();
      return -1;
    }

    //joint found
    JointData tmpData(iter->second);

    if( target < tmpData.min )
      target = tmpData.min;
    if( target > tmpData.max )
      target = tmpData.max;

    tmpData.target = target;

    joints_map[joint_name] = tmpData;
    //the targets are in radians
    target_msg.data = sr_math_utils::to_rad( target );

	//	ROS_ERROR("Joint %s ,pub index %d, pub name %s", joint_name.c_str(),tmpData.publisher_index,CAN_publishers[tmpData.publisher_index].getTopic().c_str());
    CAN_publishers[tmpData.publisher_index].publish(target_msg);

    joints_map_mutex.unlock();
    return 0;
  }

  JointData CANCompatibilityArm::getJointData( std::string joint_name )
  {
    JointData noData;
    if( !joints_map_mutex.try_lock() )
      return noData;

    JointsMap::iterator iter = joints_map.find(joint_name);

    //joint found
    if( iter != joints_map.end() )
    {
      JointData tmp = JointData(iter->second);

      joints_map_mutex.unlock();
      return tmp;
    }

    ROS_ERROR("Joint %s not found.", joint_name.c_str());
    joints_map_mutex.unlock();
    return noData;
  }

  SRArticulatedRobot::JointsMap CANCompatibilityArm::getAllJointsData()
  {
    return joints_map;
  }

  short CANCompatibilityArm::setContrl( std::string contrlr_name, JointControllerData ctrlr_data )
  {
    ROS_WARN("The set Controller function is not implemented in the CAN compatibility wrapper, please use the provided services directly.");
    return 0;
  }

  JointControllerData CANCompatibilityArm::getContrl( std::string contrlr_name )
  {
    JointControllerData no_result;
    ROS_WARN("The get Controller function is not implemented in the CAN compatibility wrapper, please use the provided services directly.");
    return no_result;
  }

  short CANCompatibilityArm::setConfig( std::vector<std::string> myConfig )
  {
    ROS_WARN("The set config function is not implemented.");
    return 0;
  }

  void CANCompatibilityArm::getConfig( std::string joint_name )
  {
    ROS_WARN("The get config function is not implemented.");
  }

  std::vector<DiagnosticData> CANCompatibilityArm::getDiagnostics()
  {
    std::vector<DiagnosticData> returnVect;
    return returnVect;
  }

  void CANCompatibilityArm::joint_states_callback(const sensor_msgs::JointStateConstPtr& msg)
  {
    if( !joints_map_mutex.try_lock() )
      return;
    //loop on all the names in the joint_states message
    for(unsigned int index = 0; index < msg->name.size(); ++index)
    {
      std::string joint_name = msg->name[index];
      JointsMap::iterator iter = joints_map.find(joint_name);

      //not found => can be a joint from the arm / hand
      if(iter == joints_map.end())
        continue;

      //joint found
      JointData tmpData(iter->second);

			tmpData.position = sr_math_utils::to_degrees(msg->position[index]);
			tmpData.force = msg->effort[index];
			tmpData.velocity = msg->velocity[index];
			joints_map[joint_name] = tmpData;

    }

    joints_map_mutex.unlock();
  }

} //end namespace


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

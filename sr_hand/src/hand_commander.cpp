/**
 * @file   hand_commander.cpp
 * @author Toni Oliver <toni@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Nov 08 15:34:37 2012
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
 * @brief  This is a library that offers a simple interface to send commands to hand joints.
 * It is compatible with the Shadow Robot CAN hand and ethercat hand.
 * It allows the user not worry about the name of the currently running controllers.
 * Only position control is allowed (targets must represent angles).
 *
 *
 */

#include <sr_hand/hand_commander.hpp>
#include <pr2_mechanism_msgs/ListControllers.h>
#include <sr_robot_msgs/sendupdate.h>
#include <std_msgs/Float64.h>
#include <boost/algorithm/string.hpp>

namespace shadowrobot
{

const double HandCommander::TIMEOUT_TO_DETECT_CONTROLLER_MANAGER = 3.0;

HandCommander::HandCommander():
    hand_type(shadowhandRosLib::UNKNOWN),
    ethercat_controllers_found(false)
{
  //We use the presence of the pr2_controller_manager/list_controllers service to detect that the hand is ethercat
  if(ros::service::waitForService("pr2_controller_manager/list_controllers", ros::Duration(TIMEOUT_TO_DETECT_CONTROLLER_MANAGER)))
  {
    hand_type = shadowhandRosLib::ETHERCAT;
    initializeEthercatHand();
    ROS_INFO("HandCommander library: ETHERCAT hand detected");
  }
  else
  {
    hand_type = shadowhandRosLib::CAN;
    sr_hand_target_pub = node_.advertise<sr_robot_msgs::sendupdate>("/srh/sendupdate", 2);
    ROS_INFO("HandCommander library: CAN hand detected");
  }
}

HandCommander::~HandCommander()
{
}

void HandCommander::initializeEthercatHand()
{
  sr_hand_target_pub_map.clear();

  ros::ServiceClient controller_list_client = node_.serviceClient<pr2_mechanism_msgs::ListControllers>("pr2_controller_manager/list_controllers");

  pr2_mechanism_msgs::ListControllers controller_list;
  std::string controlled_joint_name;

  controller_list_client.call(controller_list);

  for (size_t i=0;i<controller_list.response.controllers.size() ;i++ )
  {
    if(controller_list.response.state[i]=="running")
    {
      std::string controller = controller_list.response.controllers[i];
      if (node_.getParam("/"+controller+"/joint", controlled_joint_name))
      {
        ROS_DEBUG("controller %d:%s controls joint %s\n",
            (int)i,controller.c_str(),controlled_joint_name.c_str());
        sr_hand_target_pub_map[controlled_joint_name]
            = node_.advertise<std_msgs::Float64>(controller+"/command", 2);
        ethercat_controllers_found = true;
        sr_hand_sub_topics[controlled_joint_name] = "/"+ controller+"/state";
      }
    }
  }

}

void HandCommander::sendCommands(std::vector<sr_robot_msgs::joint> joint_vector)
{
  if(hand_type == shadowhandRosLib::ETHERCAT)
  {
    if(!ethercat_controllers_found)
    {
      initializeEthercatHand();
      // No controllers we found so bail out
      if (!ethercat_controllers_found)
          return;
    }
    for(size_t i = 0; i < joint_vector.size(); ++i)
    {
      std_msgs::Float64 target;
      target.data = joint_vector.at(i).joint_target * M_PI/180.0;
      sr_hand_target_pub_map[joint_vector.at(i).joint_name].publish(target);
    }
  }
  else
  {
    sr_robot_msgs::sendupdate sendupdate_msg;
    sendupdate_msg.sendupdate_length = joint_vector.size();
    sendupdate_msg.sendupdate_list = joint_vector;

    sr_hand_target_pub.publish(sendupdate_msg);
  }
}

  std::string HandCommander::get_controller_state_topic(std::string joint_name)
  {
    std::string topic;

    if(hand_type == shadowhandRosLib::ETHERCAT)
    {
      //urdf names are upper case
      boost::algorithm::to_upper(joint_name);
      std::map<std::string, std::string>::iterator it = sr_hand_sub_topics.find(joint_name);
      if( it != sr_hand_sub_topics.end() )
      {
        topic = it->second;
      }
      else
      {
        ROS_ERROR_STREAM(" Controller for joint " << joint_name << " not found.");
      }
    }
    else
    {
      topic = "/shadowhand_data";
    }

    return topic;
  }
}


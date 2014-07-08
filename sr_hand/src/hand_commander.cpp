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
#include <controller_manager_msgs/ListControllers.h>
#include <sr_robot_msgs/sendupdate.h>
#include <std_msgs/Float64.h>
#include <boost/algorithm/string.hpp>

namespace shadowrobot
{

  const double HandCommander::TIMEOUT_TO_DETECT_CONTROLLER_MANAGER = 3.0;

  HandCommander::HandCommander(const std::string& ns):
    node_(ns),
    hand_type(shadowhandRosLib::UNKNOWN),
    ethercat_controllers_found(false)
  {
    //Get the urdf model from the parameter server
    // this is used for returning min and max for joints for example.
    std::string robot_desc_string;
    node_.param("sh_description", robot_desc_string, std::string());
    urdf::Model robot_model;
    if (!robot_model.initString(robot_desc_string))
    {
      ROS_WARN("Failed to parse urdf file - trying with robot_description instead of sh_description.");

      node_.param("robot_description", robot_desc_string, std::string());
      if (!robot_model.initString(robot_desc_string))
      {
        ROS_ERROR_STREAM("Couldn't parse the urdf file on sh_description or on robot_description (namespace=" << node_.getNamespace() << ")");
        return;
      }
    }

    all_joints = robot_model.joints_;

    //We use the presence of the controller_manager/list_controllers service to detect that the hand is ethercat
    //We look for the manager in the robots namespace (that of node_ not the process).
    if(ros::service::waitForService(node_.getNamespace() + "/controller_manager/list_controllers", ros::Duration(TIMEOUT_TO_DETECT_CONTROLLER_MANAGER)))
    {
      hand_type = shadowhandRosLib::ETHERCAT;
      initializeEthercatHand();
      ROS_INFO_STREAM("HandCommander library: ETHERCAT hand detected in " << node_.getNamespace());
    }
    else
    {
      hand_type = shadowhandRosLib::CAN;
      sr_hand_target_pub = node_.advertise<sr_robot_msgs::sendupdate>("srh/sendupdate", 2);
      ROS_INFO_STREAM("HandCommander library: CAN hand detected in " << node_.getNamespace());
    }
  }

  HandCommander::~HandCommander()
  {
  }

  void HandCommander::initializeEthercatHand()
  {
    sr_hand_target_pub_map.clear();

    ros::ServiceClient controller_list_client = node_.serviceClient<controller_manager_msgs::ListControllers>("controller_manager/list_controllers");

    controller_manager_msgs::ListControllers controller_list;
    std::string controlled_joint_name;

    controller_list_client.call(controller_list);
    for (size_t i=0;i<controller_list.response.controller.size() ;i++ )
    {
      if(controller_list.response.controller[i].state=="running")
      {
        std::string controller = node_.resolveName(controller_list.response.controller[i].name);
        if (node_.getParam(controller+"/joint", controlled_joint_name))
        {
          ROS_DEBUG("controller %d:%s controls joint %s\n",
                    (int)i,controller.c_str(),controlled_joint_name.c_str());
          sr_hand_target_pub_map[controlled_joint_name]
            = node_.advertise<std_msgs::Float64>(controller+"/command", 2);
          ethercat_controllers_found = true;
          sr_hand_sub_topics[controlled_joint_name] = controller+"/state";
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
        boost::algorithm::to_upper(joint_vector.at(i).joint_name);
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

  std::pair<double, double> HandCommander::get_min_max(std::string joint_name)
  {
    //needs to get min max for J1 and J2 if J0
    std::vector<std::string> joint_names, split_name;
    boost::split( split_name, joint_name, boost::is_any_of("0") );
    if( split_name.size() == 1)
    {
      //not a J0
      joint_names.push_back(joint_name);
    }
    else
    {
      //this is a J0, push J1 and J2
      joint_names.push_back(split_name[0] + "1");
      joint_names.push_back(split_name[0] + "2");
    }


    std::pair<double, double> min_max;
    for( size_t i = 0; i < joint_names.size(); ++i)
    {
      std::string jn = joint_names[i];

      //urdf names are upper case
      boost::algorithm::to_upper(jn);
      std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it = all_joints.find(jn);

      if( it != all_joints.end() )
      {
        min_max.first += it->second->limits->lower;
        min_max.second += it->second->limits->upper;
      }
      else
      {
        ROS_ERROR_STREAM("Joint " << jn << " not found in the urdf description.");
      }
    }

    return min_max;
  }

  std::vector<std::string> HandCommander::get_all_joints()
  {
    std::vector<std::string> all_joints_names;
    std::map<std::string, std::string>::iterator it;

    for( it = sr_hand_sub_topics.begin(); it != sr_hand_sub_topics.end(); ++it )
    {
      // all Hand joint names have a length of 4...
      //The other way would be to check if the name is in a list
      // of possible names. Not sure what's best.
      if(it->first.size() == 4)
        all_joints_names.push_back(it->first);
    }

    return all_joints_names;
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
      topic = "shadowhand_data";
    }

    return topic;
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

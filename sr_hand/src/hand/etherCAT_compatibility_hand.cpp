/**
 * @file   virtual_shadowhand.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Tue May 25 17:50:42 2010
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
 * @brief
 *
 *
 */

#include "sr_hand/hand/etherCAT_compatibility_hand.hpp"
#include <sr_utilities/sr_math_utils.hpp>

#include <time.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <std_msgs/Float64.h>
#include <control_msgs/JointControllerState.h>
#include <sr_robot_msgs/JointControllerState.h>

namespace shadowrobot
{
  ROS_DEPRECATED EtherCATCompatibilityHand::EtherCATCompatibilityHand() :
    SRArticulatedRobot(), n_tilde("~")
  {
    ROS_WARN("This interface is deprecated, you should probably use the interface provided by the etherCAT driver directly.");

    initializeMap();

    joint_state_subscriber = node.subscribe("/joint_states", 2, &EtherCATCompatibilityHand::joint_states_callback, this);

  }

  EtherCATCompatibilityHand::~EtherCATCompatibilityHand()
  {
  }

  void EtherCATCompatibilityHand::initializeMap()
  {
    joints_map_mutex.lock();
    JointData tmpData;
    JointData tmpDataZero;
    tmpDataZero.isJointZero = 1;
    tmpDataZero.max = 180.0;
    tmpData.max=90.0;

    std::string full_topic = "";

    full_topic = findControllerTopicName("ffj0");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    int tmp_index = 0;
    tmpDataZero.publisher_index = tmp_index;
    joints_map["FFJ0"] = tmpDataZero;
    joints_map["FFJ1"] = tmpData;
    joints_map["FFJ2"] = tmpData;

    full_topic = findControllerTopicName("ffj3");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["FFJ3"] = tmpData;

    tmpData.min = -20.0;
    tmpData.max = 20.0;
    full_topic = findControllerTopicName("ffj4");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["FFJ4"] = tmpData;

    full_topic = findControllerTopicName("mfj0");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpDataZero.publisher_index = tmp_index;
    joints_map["MFJ0"] = tmpDataZero;

    tmpData.min = 0.0;
    tmpData.max = 90.0;
    joints_map["MFJ1"] = tmpData;
    joints_map["MFJ2"] = tmpData;

    full_topic = findControllerTopicName("mfj3");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["MFJ3"] = tmpData;

    tmpData.min = -20.0;
    tmpData.max = 20.0;
    full_topic = findControllerTopicName("mfj4");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["MFJ4"] = tmpData;

    full_topic = findControllerTopicName("rfj0");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpDataZero.publisher_index = tmp_index;
    joints_map["RFJ0"] = tmpDataZero;

    tmpData.min = 0.0;
    tmpData.max = 90.0;
    joints_map["RFJ1"] = tmpData;
    joints_map["RFJ2"] = tmpData;
    full_topic = findControllerTopicName("rfj3");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["RFJ3"] = tmpData;

    tmpData.min = -20.0;
    tmpData.max = 20.0;
    full_topic = findControllerTopicName("rfj4");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["RFJ4"] = tmpData;

    full_topic = findControllerTopicName("lfj0");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpDataZero.publisher_index = tmp_index;
    joints_map["LFJ0"] = tmpDataZero;

    tmpData.min = 0.0;
    tmpData.max = 90.0;
    joints_map["LFJ1"] = tmpData;
    joints_map["LFJ2"] = tmpData;

    full_topic = findControllerTopicName("lfj3");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["LFJ3"] = tmpData;

    tmpData.min = -20.0;
    tmpData.max = 20.0;
    full_topic = findControllerTopicName("lfj4");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["LFJ4"] = tmpData;

    tmpData.min = 0.0;
    tmpData.max = 45.0;
    full_topic = findControllerTopicName("lfj5");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["LFJ5"] = tmpData;

    tmpData.min = 0.0;
    tmpData.max = 90.0;
    full_topic = findControllerTopicName("thj1");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["THJ1"] = tmpData;

    tmpData.min = -40.0;
    tmpData.max = 40.0;
    full_topic = findControllerTopicName("thj2");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["THJ2"] = tmpData;

    tmpData.min = -15.0;
    tmpData.max = 15.0;
    full_topic = findControllerTopicName("thj3");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["THJ3"] = tmpData;

    tmpData.min = 0.0;
    tmpData.max = 75.0;
    full_topic = findControllerTopicName("thj4");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["THJ4"] = tmpData;

    tmpData.min = -60.0;
    tmpData.max = 60.0;
    full_topic = findControllerTopicName("thj5");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["THJ5"] = tmpData;

    tmpData.min = -30.0;
    tmpData.max = 45.0;
    full_topic = findControllerTopicName("wrj1");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["WRJ1"] = tmpData;

    tmpData.min = -30.0;
    tmpData.max = 10.0;
    full_topic = findControllerTopicName("wrj2");
    etherCAT_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
    joints_map["WRJ2"] = tmpData;

    joints_map_mutex.unlock();
  }

  std::string EtherCATCompatibilityHand::findControllerTopicName( std::string joint_name)
  {
    std::string joint_prefix;
    n_tilde.param("joint_prefix", joint_prefix, std::string(""));


    std::string controller_suffix = "position_controller";

    std::string searched_param;
    n_tilde.searchParam("controller_suffix", searched_param);
    n_tilde.param(searched_param, controller_suffix, std::string("position_controller"));

    std::string topic_prefix = "/sh_";
    std::string topic_suffix = "_" + controller_suffix + "/command";
    std::string full_topic = topic_prefix + joint_prefix + joint_name + topic_suffix;

    return full_topic;
  }

  short EtherCATCompatibilityHand::sendupdate( std::string joint_name, double target )
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

    //  ROS_ERROR("Joint %s ,pub index %d, pub name %s", joint_name.c_str(),tmpData.publisher_index,etherCAT_publishers[tmpData.publisher_index].getTopic().c_str());
    etherCAT_publishers[tmpData.publisher_index].publish(target_msg);

    joints_map_mutex.unlock();
    return 0;
  }

  JointData EtherCATCompatibilityHand::getJointData( std::string joint_name )
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

  SRArticulatedRobot::JointsMap EtherCATCompatibilityHand::getAllJointsData()
  {
    return joints_map;
  }

  short EtherCATCompatibilityHand::setContrl( std::string contrlr_name, JointControllerData ctrlr_data )
  {
    ROS_WARN("The set Controller function is not implemented in the EtherCAT compatibility wrapper, please use the provided services directly.");
    return 0;
  }

  JointControllerData EtherCATCompatibilityHand::getContrl( std::string contrlr_name )
  {
    JointControllerData no_result;
    ROS_WARN("The get Controller function is not implemented in the EtherCAT compatibility wrapper, please use the provided services directly.");
    return no_result;
  }

  short EtherCATCompatibilityHand::setConfig( std::vector<std::string> myConfig )
  {
    ROS_WARN("The set config function is not implemented.");
    return 0;
  }

  void EtherCATCompatibilityHand::getConfig( std::string joint_name )
  {
    ROS_WARN("The get config function is not implemented.");
  }

  std::vector<DiagnosticData> EtherCATCompatibilityHand::getDiagnostics()
  {
    std::vector<DiagnosticData> returnVect;
    return returnVect;
  }

  void EtherCATCompatibilityHand::joint_states_callback(const sensor_msgs::JointStateConstPtr& msg)
  {
    if( !joints_map_mutex.try_lock() )
      return;
    std::string fj0char;
    bool fj0flag=false;
    double fj0pos,fj0vel,fj0eff;
    //loop on all the names in the joint_states message
    for(unsigned int index = 0; index < msg->name.size(); ++index)
      {
	std::string joint_name = msg->name[index];
	JointsMap::iterator iter = joints_map.find(joint_name);

	//not found => can be a joint from the arm 
	if(iter == joints_map.end())
	  {
	    continue;
	  }
	else
	  {
	    //joint found but may be a FJ1 or FJ2 to combine into FJ0
	    if( joint_name.find("FJ1")!=std::string::npos)
	      {
		//ROS_INFO("FJ1found");
		if(fj0flag && joint_name[0]==fj0char[0]) //J2 already found for SAME joint
		  {
		    std::string j0name=fj0char+"FJ0"; 
		    JointsMap::iterator myiter = joints_map.find(j0name);
		    if(myiter == joints_map.end()) // joint is not existing
		      continue;
		    else
		      {
			JointData tmpDatazero(myiter->second);
			tmpDatazero.position = fj0pos+ sr_math_utils::to_degrees(msg->position[index]);
			tmpDatazero.velocity = fj0vel + msg->effort[index];
			tmpDatazero.force = fj0eff + msg->velocity[index];
			joints_map[j0name] = tmpDatazero;
        
			fj0pos=0;
			fj0vel=0;
			fj0eff=0;
			fj0char.clear();
			fj0flag=false;
		      }
		  }
		else
		  {
		    fj0pos=sr_math_utils::to_degrees(msg->position[index]);
		    fj0eff=msg->effort[index];
		    fj0vel=msg->velocity[index];
		    fj0char.push_back(joint_name[0]);
		    fj0flag=true;
		    //ROS_INFO("FFJ1found%s, j0char:%s,flag:%d,equal:%d",joint_name.c_str(),fj0char.c_str(),fj0flag==true?1:0,joint_name[0]==fj0char[0]?1:0);
		  }
	      }
	    else if( joint_name.find("FJ2")!=std::string::npos)
	      {
		//ROS_INFO("FJ2found");
		if(fj0flag && joint_name[0]==fj0char[0]) //J1 already found for SAME joint
		  {
		    std::string j0name=fj0char+"FJ0";
		    JointsMap::iterator myiter = joints_map.find(j0name);
		    if(myiter == joints_map.end()) // joint is not existing
		      continue;
		    else
		      {
			JointData tmpDatazero(myiter->second);
			tmpDatazero.position = fj0pos+ sr_math_utils::to_degrees(msg->position[index]);
			tmpDatazero.force = fj0vel + msg->effort[index];
			tmpDatazero.velocity = fj0eff + msg->velocity[index];
			joints_map[j0name] = tmpDatazero;
          
			fj0pos=0;
			fj0vel=0;
			fj0eff=0;
			fj0char.clear();
			fj0flag=false;
		      }
		  }
		else
		  {
		    fj0pos=sr_math_utils::to_degrees(msg->position[index]);
		    fj0eff=msg->effort[index];
		    fj0vel=msg->velocity[index];
		    fj0char.push_back(joint_name[0]);
		    fj0flag=true;
		    //ROS_INFO("FFJ2found%s, j0char:%s,flag:%d,equal:%d",joint_name.c_str(),fj0char.c_str(),fj0flag==true?1:0,joint_name[0]==fj0char[0]?1:0);
		  }
	      }
	    // any way do fill the found joint data and update the joint map.
	    JointData tmpData(iter->second);
	    tmpData.position = sr_math_utils::to_degrees(msg->position[index]);
	    tmpData.force = msg->effort[index];
	    tmpData.velocity = msg->velocity[index];
	    joints_map[joint_name] = tmpData;
	  }  
    
      }

    joints_map_mutex.unlock();
  }

} //end namespace


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

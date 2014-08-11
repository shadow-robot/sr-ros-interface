/**
 * @file   sr_subscriber.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Mar 25 15:34:37 2010
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
 * @brief  The role of this ROS subscriber is to receive commands
 * messages (sent by a publisher) and pass them to the hand after
 * translating them to the correct format.
 *
 *
 */

#include <iostream>

#include "sr_hand/sr_subscriber.h"
#include <boost/algorithm/string.hpp>
#include <sr_utilities/sr_math_utils.hpp>
#include <boost/algorithm/string.hpp>

using namespace std;

namespace shadowrobot
{

/////////////////////////////////
//    CONSTRUCTOR/DESTRUCTOR   //
/////////////////////////////////

SRSubscriber::SRSubscriber( boost::shared_ptr<SRArticulatedRobot> sr_art_robot ) :
    n_tilde("~")
{
    sr_articulated_robot = sr_art_robot;
    ///////
    // Initialize the subscribers
    //////
    SRSubscriber::init();
}

SRSubscriber::~SRSubscriber()
{
}

void SRSubscriber::init()
{
    // subscribe to sendupdate topic, set sendupdateCallback function
    std::string prefix;
    std::string searched_param;
    n_tilde.searchParam("shadowhand_prefix", searched_param);
    n_tilde.param(searched_param, prefix, std::string());
    std::string full_topic = prefix + "sendupdate";

    sendupdate_sub = node.subscribe(full_topic, 2, &SRSubscriber::sendupdateCallback, this);

    // subscribe to contrlr topic, set contrlrCallback function
    full_topic = prefix + "contrlr";
    contrlr_sub = node.subscribe(full_topic, 2, &SRSubscriber::contrlrCallback, this);

    // subscribe to config topic, set configCallback function
    full_topic = prefix + "config";
    config_sub = node.subscribe(full_topic, 2, &SRSubscriber::configCallback, this);

    // subscribe to the new ethercat like topics: one topic per joint
    for(SRArticulatedRobot::JointsMap::iterator joint = sr_articulated_robot->joints_map.begin() ;
        joint != sr_articulated_robot->joints_map.end(); ++joint)
    {
      controllers_sub.push_back( node.subscribe<std_msgs::Float64>("sh_"+boost::to_lower_copy(joint->first+"_position_controller/command"), 2, boost::bind(&SRSubscriber::cmd_callback, this, _1, joint->first) ) );
    }
}

/////////////////////////////////
//         CALLBACK            //
/////////////////////////////////
void SRSubscriber::sendupdateCallback( const sr_robot_msgs::sendupdateConstPtr& msg )
{
    //loop on all the sendupdate messages received (if > 0)
    int sendupdate_length = msg->sendupdate_length;
    if( sendupdate_length == 0 )
    {
        ROS_DEBUG("Received empty sendupdate command.");
        return;
    }
    //OK, not empty => loop to process all the sendupdate messages
    for( unsigned short index_msg = 0; index_msg < msg->sendupdate_length; ++index_msg )
    {
        float target = msg->sendupdate_list[index_msg].joint_target;
        string sensor_name = msg->sendupdate_list[index_msg].joint_name;

        ROS_DEBUG("Received sendupdate Command [%s : %f]", sensor_name.c_str(), target);
        sr_articulated_robot->sendupdate(sensor_name, (double)target);
    }

}

void SRSubscriber::cmd_callback( const std_msgs::Float64ConstPtr& msg, std::string& joint_name )
{
  //converting to degrees as the old can interface was expecting degrees
  sr_articulated_robot->sendupdate(joint_name, sr_math_utils::to_degrees(msg->data));
}

void SRSubscriber::contrlrCallback( const sr_robot_msgs::contrlrConstPtr& msg )
{

    vector<string> list_of_parameters = msg->list_of_parameters;

    JointControllerData ctrl_data;

    //parses all the parameters transmitted in the msg
    for( unsigned int index_param = 0; index_param < msg->length_of_list; ++index_param )
    {
        //split the string (around ":")
        vector<string> splitted_string;
        boost::split(splitted_string, msg->list_of_parameters[index_param], boost::is_any_of(":"));

        Parameters param;
        param.name = splitted_string[0];
        param.value = splitted_string[1];

        ctrl_data.data.push_back(param);
    }

    sr_articulated_robot->setContrl(msg->contrlr_name, ctrl_data);
}

void SRSubscriber::configCallback( const sr_robot_msgs::configConstPtr& msg )
{
    ROS_ERROR("Configuration command callback not implemented yet");
}

} // end namespace

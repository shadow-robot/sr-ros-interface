/**
 * @file   shadowhand_to_cybergrasp_remapper.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu May 13 09:44:52 2010
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
 * @brief This program remapps the force information contained in
 * /joint_states coming from the hand to the /cybergraspforces topic
 * used to control the cybergrasp.
 *
 *
 */

//ROS include
#include <ros/ros.h>

//generic include
#include <string>

//own .h
#include "sr_remappers/shadowhand_to_cyberglove_remapper.h"
#include <sr_robot_msgs/sendupdate.h>
#include <sr_robot_msgs/joint.h>
using namespace ros;

namespace shadowhand_to_cyberglove_remapper
{

const unsigned int ShadowhandToCybergloveRemapper::number_hand_joints = 20;

ShadowhandToCybergloveRemapper::ShadowhandToCybergloveRemapper() :
    n_tilde("~")
{
    joints_names.resize(number_hand_joints);
    ShadowhandToCybergloveRemapper::init_names();

    std::string param;
    std::string path;
    n_tilde.searchParam("cyberglove_mapping_path", param);
    n_tilde.param(param, path, std::string());
    calibration_parser = new CalibrationParser(path);
    ROS_INFO("Mapping file loaded for the Cyberglove: %s", path.c_str());

    std::string prefix;
    std::string searched_param;
    n_tilde.searchParam("cyberglove_prefix", searched_param);
    n_tilde.param(searched_param, prefix, std::string());

    std::string full_topic = prefix + "/calibrated/joint_states";

    cyberglove_jointstates_sub = node.subscribe(full_topic, 10, &ShadowhandToCybergloveRemapper::jointstatesCallback, this);

    n_tilde.searchParam("sendupdate_prefix", searched_param);
    n_tilde.param(searched_param, prefix, std::string());
    full_topic = prefix + "sendupdate";

    shadowhand_pub = node.advertise<sr_robot_msgs::sendupdate> (full_topic, 5);
}

void ShadowhandToCybergloveRemapper::init_names()
{
    joints_names[0] = "THJ1";
    joints_names[1] = "THJ2";
    joints_names[2] = "THJ3";
    joints_names[3] = "THJ4";
    joints_names[4] = "THJ5";
    joints_names[5] = "FFJ0";
    joints_names[6] = "FFJ3";
    joints_names[7] = "FFJ4";
    joints_names[8] = "MFJ0";
    joints_names[9] = "MFJ3";
    joints_names[10] = "MFJ4";
    joints_names[11] = "RFJ0";
    joints_names[12] = "RFJ3";
    joints_names[13] = "RFJ4";
    joints_names[14] = "LFJ0";
    joints_names[15] = "LFJ3";
    joints_names[16] = "LFJ4";
    joints_names[17] = "LFJ5";
    joints_names[18] = "WRJ1";
    joints_names[19] = "WRJ2";
}

void ShadowhandToCybergloveRemapper::jointstatesCallback( const sensor_msgs::JointStateConstPtr& msg )
{
    sr_robot_msgs::joint joint;
    sr_robot_msgs::sendupdate pub;

    //Do conversion
    std::vector<double> vect = calibration_parser->get_remapped_vector(msg->position);
    //Generate sendupdate message
    pub.sendupdate_length = number_hand_joints;

    std::vector<sr_robot_msgs::joint> table(number_hand_joints);
    for(unsigned int i = 0; i < number_hand_joints; ++i )
    {
        joint.joint_name = joints_names[i];
        joint.joint_target = vect[i];
        table[i] = joint;
    }
    pub.sendupdate_length = number_hand_joints;
    pub.sendupdate_list = table;
    shadowhand_pub.publish(pub);
}
}//end namespace

/**
 * @file   sr_publisher.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Mar 25 15:36:41 2010
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
 * @brief The goal of this ROS publisher is to publish relevant data
 * concerning the hand at a regular time interval.
 * Those data are (not exhaustive): positions, targets, temperatures,
 * currents, forces, error flags, ...
 *
 *
 */

//ROS include
#include <ros/ros.h>

//messages
#include <sr_robot_msgs/joints_data.h>
#include <sr_robot_msgs/joint.h>
#include <sensor_msgs/JointState.h>

//generic C/C++ include
#include <vector>
#include <string>
#include <sstream>

#include "sr_hand/sr_publisher.h"

using namespace ros;
using namespace shadowrobot;

namespace shadowrobot
{
/////////////////////////////////
//    CONSTRUCTOR/DESTRUCTOR   //
/////////////////////////////////
SRPublisher::SRPublisher( boost::shared_ptr<SRArticulatedRobot> sh ) :
    n_tilde("~"), publish_rate(0.0)
{
    sr_articulated_robot = sh;

    // set publish frequency
    double publish_freq;
    n_tilde.param("publish_frequency", publish_freq, 50.0);
    publish_rate = Rate(publish_freq);

    //publishes JointState messages for the robot_state_publisher
    std::string prefix;
    std::string searched_param;
    n_tilde.searchParam("shadowhand_prefix", searched_param);
    n_tilde.param(searched_param, prefix, std::string());
    std::string full_topic = prefix + "position/joint_states";
    sr_jointstate_pos_pub = node.advertise<sensor_msgs::JointState> (full_topic, 2);
    full_topic = prefix + "target/joint_states";
    sr_jointstate_target_pub = node.advertise<sensor_msgs::JointState> (full_topic, 2);

    //publishes standard joints data (pos, targets, temp, current, ...)
    full_topic = prefix + "shadowhand_data";
    sr_pub = node.advertise<sr_robot_msgs::joints_data> (full_topic, 2);
}

SRPublisher::~SRPublisher()
{
    //if( shadowhand != NULL )
    // delete shadowhand;
}

/////////////////////////////////
//       PUBLISH METHOD        //
/////////////////////////////////
void SRPublisher::publish()
{
    SRArticulatedRobot::JointsMap joints_map = sr_articulated_robot->getAllJointsData();

    sr_robot_msgs::joints_data msg;
    std::vector<sr_robot_msgs::joint> jointVector;

    sensor_msgs::JointState jointstate_pos_msg;
    sensor_msgs::JointState jointstate_target_msg;

    ros::Time now = ros::Time::now();
    jointstate_pos_msg.header.stamp = now; 
    jointstate_target_msg.header.stamp = now;

    for( SRArticulatedRobot::JointsMap::const_iterator it = joints_map.begin(); it != joints_map.end(); ++it )
    {
        sr_robot_msgs::joint joint;
        JointData currentData = it->second;

        //compute the angular velocity of the joint
        if(currentData.last_pos_time.toSec() != 0.0)
        {
            currentData.velocity = (currentData.position - currentData.last_pos);
            currentData.velocity /= (now - currentData.last_pos_time).toSec();
            ROS_DEBUG("Velocity = (%f - %f)/(%f) = %f", currentData.position, currentData.last_pos, (now - currentData.last_pos_time).toSec(), currentData.velocity);
        }

        joint.joint_name = it->first;
        jointstate_pos_msg.name.push_back(it->first);
        jointstate_target_msg.name.push_back(it->first);

        jointstate_target_msg.position.push_back(toRad(currentData.target));
        jointstate_target_msg.velocity.push_back(0.0);
        jointstate_target_msg.effort.push_back(0.0);

        jointstate_pos_msg.position.push_back(toRad(currentData.position));
        jointstate_pos_msg.velocity.push_back(currentData.velocity);
        jointstate_pos_msg.effort.push_back(currentData.force);

        joint.joint_position = currentData.position;
        joint.joint_target = currentData.target;
        joint.joint_torque = currentData.force;
        joint.joint_temperature = currentData.temperature;
        joint.joint_current = currentData.current;

        //update data for the velocity
        currentData.last_pos_time = now;
        currentData.last_pos = currentData.position;

        sr_articulated_robot->joints_map_mutex.lock();
        sr_articulated_robot->joints_map[it->first] = JointData(currentData);
        sr_articulated_robot->joints_map_mutex.unlock();

        ROS_DEBUG("last_pos_time[%s] = %f / %f", it->first.c_str(), currentData.last_pos_time.toSec(), joints_map[it->first].last_pos_time.toSec());

        jointVector.push_back(joint);
    }

    msg.joints_list_length = jointVector.size();
    msg.joints_list = jointVector;

    //publish standard data (pos, target, current, temp, force, ...)
    sr_pub.publish(msg);
    //publish JointState position message
    sr_jointstate_pos_pub.publish(jointstate_pos_msg);
    //publish JointState target message
    sr_jointstate_target_pub.publish(jointstate_target_msg);

    ros::spinOnce();
    publish_rate.sleep();
}

}// end namespace



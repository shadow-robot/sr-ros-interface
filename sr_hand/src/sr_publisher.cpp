/**
 * @file   sr_publisher.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Mar 25 15:36:41 2010
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
#include <sr_hand/joints_data.h>
#include <sr_hand/joint.h>
#include <sensor_msgs/JointState.h>

//generic C/C++ include
#include <vector>
#include <string>
#include <sstream>

#include "sr_hand/sr_publisher.h"

using namespace ros;
using namespace shadowhand;

namespace shadowhand_publisher{
  /////////////////////////////////
  //    CONSTRUCTOR/DESTRUCTOR   //
  /////////////////////////////////
  ShadowhandPublisher::ShadowhandPublisher(boost::shared_ptr<Shadowhand> sh)
    : n_tilde("~"), publish_rate(0.0)
  {
    shadowhand = sh;

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
    shadowhand_jointstate_pos_pub = node.advertise<sensor_msgs::JointState>(full_topic, 2);
    full_topic = prefix + "target/joint_states";
    shadowhand_jointstate_target_pub = node.advertise<sensor_msgs::JointState>(full_topic, 2);

    //publishes standard joints data (pos, targets, temp, current, ...)
    full_topic = prefix + "shadowhand_data";
    shadowhand_pub = node.advertise<sr_hand::joints_data>(full_topic, 2);
  }

  ShadowhandPublisher:: ~ShadowhandPublisher()
  {
    //if( shadowhand != NULL )
     // delete shadowhand;
  }

  /////////////////////////////////
  //       PUBLISH METHOD        //
  /////////////////////////////////
  void ShadowhandPublisher::publish()
  {
    Shadowhand::JointsMap joints_map = shadowhand->getAllJointsData();

    sr_hand::joints_data msg;
    std::vector<sr_hand::joint> jointVector;

    sensor_msgs::JointState jointstate_pos_msg;
    sensor_msgs::JointState jointstate_target_msg;

    jointstate_pos_msg.header.stamp = ros::Time::now();
    jointstate_target_msg.header.stamp = ros::Time::now();

    for(Shadowhand::JointsMap::const_iterator it = joints_map.begin(); it != joints_map.end(); ++it)
      {
	sr_hand::joint joint;
	JointData currentData = it->second;

	joint.joint_name = it->first;
	jointstate_pos_msg.name.push_back(it->first);
        jointstate_target_msg.name.push_back(it->first);

        jointstate_target_msg.position.push_back(toRad(currentData.target));
        jointstate_target_msg.velocity.push_back(0.0);
        jointstate_target_msg.effort.push_back(0.0);

	jointstate_pos_msg.position.push_back(toRad(currentData.position));
	jointstate_pos_msg.velocity.push_back(0.0);
	jointstate_pos_msg.effort.push_back(currentData.force);

	joint.joint_position = currentData.position;
	joint.joint_target = currentData.target;
	joint.joint_torque = currentData.force;
	joint.joint_temperature = currentData.temperature;
	joint.joint_current = currentData.current;
	
	jointVector.push_back(joint);
      }

    msg.joints_list_length = jointVector.size();
    msg.joints_list = jointVector;

    //publish standard data (pos, target, current, temp, force, ...)
    shadowhand_pub.publish(msg);
    //publish JointState position message
    shadowhand_jointstate_pos_pub.publish(jointstate_pos_msg);
    //publish JointState target message
    shadowhand_jointstate_target_pub.publish(jointstate_target_msg);
    
    ros::spinOnce();
    publish_rate.sleep();
  }

}// end namespace



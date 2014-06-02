#pragma once

#include "sr_standalone/standalone.hpp"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace std;

namespace shadow_robot_standalone
{

class ShadowHand::SrRosWrapper
{
public:
  SrRosWrapper();
  ~SrRosWrapper();

protected:
  // fire up the ROS node
  void init(void);

  void callback(const sensor_msgs::JointStateConstPtr& msg);

public:
  JointStates joint_states_;
  vector<Tactile> tactiles_;

  ControlType control_type_;

  ros::NodeHandle nh_;
  ros::NodeHandle n_tilde_;
  ros::Subscriber joint_states_sub_;
};

} // namespace

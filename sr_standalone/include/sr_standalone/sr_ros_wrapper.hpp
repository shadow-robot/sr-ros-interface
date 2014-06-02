#pragma once

#include "sr_standalone/shadow_hand.hpp"
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace std;

namespace shadow_robot_standalone
{

class ShadowHand::SrRosWrapper
{
public:
  SrRosWrapper(int argc, char **argv);
  ~SrRosWrapper();

protected:
  // fire up the ROS node
  void init(int argc, char **argv);

  void callback(const sensor_msgs::JointStateConstPtr& msg);

public:
  JointStates joint_states_;
  vector<Tactile> tactiles_;

  ControlType control_type_;

  boost::scoped_ptr<ros::NodeHandle> nh_;
  boost::scoped_ptr<ros::NodeHandle> n_tilde_;
  ros::Subscriber joint_states_sub_;
};

} // namespace

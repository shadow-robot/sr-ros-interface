#pragma once

#include "sr_standalone/shadow_hand.hpp"
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sr_robot_msgs/ShadowPST.h>

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

  void joint_state_cb(const sensor_msgs::JointStateConstPtr& msg);
  void tactile_cb(const sr_robot_msgs::ShadowPSTConstPtr& msg);
  void spin(void);

public:
  JointStates joint_states_;
  std::vector<Tactile> tactiles_;

  ControlType control_type_;

  boost::scoped_ptr<ros::NodeHandle> nh_;
  boost::scoped_ptr<ros::NodeHandle> n_tilde_;

  ros::Subscriber joint_states_sub_;
  ros::Subscriber tactile_sub_;

  boost::scoped_ptr<boost::thread> spin_thread_;

  bool time_to_quit_;

  double ros_spin_rate_;
};

} // namespace

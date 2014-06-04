#pragma once

#include "sr_standalone/shadow_hand.hpp"
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sr_robot_msgs/ControlType.h>
#include <sr_robot_msgs/ChangeControlType.h>
#include <sr_robot_msgs/ChangeControlTypeRequest.h>
#include <sr_robot_msgs/ChangeControlTypeResponse.h>
#include <sr_robot_msgs/BiotacAll.h>

namespace shadow_robot_standalone
{

class ShadowHand::SrRosWrapper
{
public:
  SrRosWrapper(int argc, char **argv);
  ~SrRosWrapper();

  bool get_control_type(ControlType & current_ctrl_type);
  bool set_control_type(const ControlType & new_ctrl_type);

  void send_position(const std::string &joint_name, double target);
  void send_torque(const std::string &joint_name, double target);

protected:
  // fire up the ROS node
  void init(int argc, char **argv);

  void joint_state_cb(const sensor_msgs::JointStateConstPtr& msg);
  void tactile_cb(const sr_robot_msgs::BiotacAllConstPtr& msg);
  void spin(void);

public:
  JointStates joint_states_;
  std::vector<Tactile> tactiles_;

  boost::scoped_ptr<ros::NodeHandle> nh_;
  boost::scoped_ptr<ros::NodeHandle> n_tilde_;

  ros::Subscriber joint_states_sub_;
  ros::Subscriber tactile_sub_;

  boost::scoped_ptr<boost::thread> spin_thread_;

  bool time_to_quit_;

  double ros_spin_rate_;
};

} // namespace

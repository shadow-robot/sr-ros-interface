#ifndef _SR_ROS_WRAPPER_HPP_
#define _SR_ROS_WRAPPER_HPP_

#include "standalone.hpp"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

using namespace std;

namespace shadow_robot_standalone
{

class ShadowHand::SrRosWrapper
{
public:
  SrRosWrapper() : control_type_(POSITION_PWM), n_tilde_("~")
  {
    init();
  }

  ~SrRosWrapper();

  JointStates joint_states_;
  vector<Tactile> tactiles_;

  ControlType control_type_;

  ros::NodeHandle nh_, n_tilde_;
  ros::Subscriber joint_states_sub_;

  // fire up the ROS node
  void init()
  {
    const string name = "standalone_example";
    int argc = 1;
    char **argv = (char**) calloc(2, sizeof(char*) );
    argv[0] = const_cast<char*>(name.c_str());
    ros::init(argc, argv, name);

    string joint_states_topic;
    n_tilde_.searchParam("prefix", joint_states_topic);
    joint_states_topic += "position/joint_states";

    joint_states_sub_ = nh_.subscribe(joint_states_topic, 2, &SrRosWrapper::callback, this);

    while( ros::ok() )
      ros::spin();
  }

  void callback(const sensor_msgs::JointStateConstPtr& msg)
  {
    joint_states_.names      = msg->name;
    joint_states_.positions  = msg->position;
    joint_states_.velocities = msg->velocity;
    joint_states_.efforts    = msg->effort;
  }
};

} // namespace

#endif // _SR_ROS_WRAPPER_HPP_

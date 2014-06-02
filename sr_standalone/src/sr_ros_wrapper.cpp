#include "sr_standalone/sr_ros_wrapper.hpp"

using namespace std;

namespace shadow_robot_standalone
{

ShadowHand::SrRosWrapper::SrRosWrapper(int argc, char **argv)
  : control_type_(POSITION_PWM)
{
  init(argc, argv);
}

ShadowHand::SrRosWrapper::~SrRosWrapper()
{
}

// fire up the ROS node
void ShadowHand::SrRosWrapper::init(int argc, char **argv)
{
  const string node_name = "sh_standalone_node";
  ros::init(argc, argv, node_name);

  // Must call ros::init() before creating the first NodeHandle.
  nh_.reset(new ros::NodeHandle());
  n_tilde_.reset(new ros::NodeHandle("~"));

  string joint_states_topic;
  n_tilde_->searchParam("prefix", joint_states_topic);
  joint_states_topic += "position/joint_states";

  joint_states_sub_ = nh_->subscribe(joint_states_topic, 2, &SrRosWrapper::callback, this);

  while( ros::ok() )
    ros::spin();
}

void ShadowHand::SrRosWrapper::callback(const sensor_msgs::JointStateConstPtr& msg)
{
  joint_states_.names      = msg->name;
  joint_states_.positions  = msg->position;
  joint_states_.velocities = msg->velocity;
  joint_states_.efforts    = msg->effort;
}

} // namespace

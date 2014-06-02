#include "sr_standalone/sr_ros_wrapper.hpp"

using namespace std;

namespace shadow_robot_standalone
{

ShadowHand::SrRosWrapper::SrRosWrapper()
  : control_type_(POSITION_PWM),
    n_tilde_("~")
{
  init();
}

ShadowHand::SrRosWrapper::~SrRosWrapper()
{
}

// fire up the ROS node
void ShadowHand::SrRosWrapper::init(void)
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

void ShadowHand::SrRosWrapper::callback(const sensor_msgs::JointStateConstPtr& msg)
{
  joint_states_.names      = msg->name;
  joint_states_.positions  = msg->position;
  joint_states_.velocities = msg->velocity;
  joint_states_.efforts    = msg->effort;
}

} // namespace

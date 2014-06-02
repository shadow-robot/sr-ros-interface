#include "sr_standalone/sr_ros_wrapper.hpp"

using namespace std;

namespace shadow_robot_standalone
{

ShadowHand::SrRosWrapper::SrRosWrapper(int argc, char **argv)
  : control_type_(POSITION_PWM),
    time_to_quit_(false),
    ros_spin_rate_(100) // 100 hz
{
  init(argc, argv);
  spin_thread_.reset(new boost::thread(&ShadowHand::SrRosWrapper::spin, this));
}

ShadowHand::SrRosWrapper::~SrRosWrapper()
{
  time_to_quit_ = true;
  spin_thread_->join();
}

void ShadowHand::SrRosWrapper::spin(void)
{
  ros::Rate rate(ros_spin_rate_);
  while( ros::ok() && !time_to_quit_ )
  {
    ros::spinOnce();
    rate.sleep();
  }
}

void ShadowHand::SrRosWrapper::init(int argc, char **argv)
{
  const string node_name = "sh_standalone_node";
  ros::init(argc, argv, node_name);

  // Must call ros::init() before creating the first NodeHandle.
  nh_.reset(new ros::NodeHandle());
  n_tilde_.reset(new ros::NodeHandle("~"));

  string joint_states_topic;
  n_tilde_->searchParam("prefix", joint_states_topic);
  joint_states_topic += "/joint_states";
  ROS_INFO_STREAM("joint_states_topic = " << joint_states_topic);

  string tactile_topic;
  n_tilde_->searchParam("prefix", tactile_topic);
  tactile_topic += "/tactile";
  ROS_INFO_STREAM("tactile_topic = " << tactile_topic);

  joint_states_sub_ = nh_->subscribe(joint_states_topic, 1, &SrRosWrapper::joint_state_cb, this);
  tactile_sub_ = nh_->subscribe(tactile_topic, 1, &SrRosWrapper::tactile_cb, this);
}

void ShadowHand::SrRosWrapper::joint_state_cb(const sensor_msgs::JointStateConstPtr& msg)
{
  joint_states_.names      = msg->name;
  joint_states_.positions  = msg->position;
  joint_states_.velocities = msg->velocity;
  joint_states_.efforts    = msg->effort;
}

void ShadowHand::SrRosWrapper::tactile_cb(const sr_robot_msgs::ShadowPSTConstPtr& msg)
{
  tactiles_.clear();

  Tactile tactile;
  tactile.pac0 = msg->pressure[0];
  tactile.pac1 = msg->pressure[1];
  tactile.pdc  = msg->pressure[2];
  tactile.tac  = msg->pressure[3];
  tactile.tdc  = msg->pressure[4];
  // ??? How to set ???
  // tactile.electrodes[19];

  tactiles_.push_back(tactile);
}

} // namespace

#include "sr_standalone/shadow_hand.hpp"
#include "sr_standalone/sr_ros_wrapper.hpp"
#include <algorithm>

namespace shadow_robot_standalone
{

ShadowHand::ShadowHand(int argc, char** argv)
  : wrapper_(new SrRosWrapper(argc, argv))
{
}

ShadowHand::~ShadowHand()
{
}

bool ShadowHand::get_control_type(ControlType & control_type)
{
  return ( wrapper_->get_control_type(control_type) );
}

bool ShadowHand::set_control_type(ControlType control_type)
{
  return ( wrapper_->set_control_type(control_type) );
}

void ShadowHand::send_position(const std::string &joint_name, double target)
{
  wrapper_->send_position(joint_name, target);
}

void ShadowHand::send_torque(const std::string &joint_name, double target)
{
  wrapper_->send_torque(joint_name, target);
}

const JointStates & ShadowHand::get_joint_states() const
{
  return wrapper_->joint_states_;
}

const std::vector<Tactile> & ShadowHand::get_tactiles() const
{
  return wrapper_->tactiles_;
}

const std::vector<std::string> & ShadowHand::get_list_of_joints() const
{
  return wrapper_->joint_states_.names;
}

} // namespace

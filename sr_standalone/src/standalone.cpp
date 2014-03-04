#include "sr_standalone/standalone.hpp"
#include "sr_standalone/sr_ros_wrapper.hpp"
#include <algorithm>

using namespace std;


namespace shadow_robot_standalone
{

ShadowHand::ShadowHand() : srw(new SrRosWrapper()) {}

bool ShadowHand::set_control_type(ControlType control_type)
{
  srw->control_type_ = control_type;
  return true;  // TODO check if control_type was actually set at the ROS side of things
}

void ShadowHand::send_position(const string &joint_name, double target)
{
}

void ShadowHand::send_torque(const string &joint_name, double target)
{
}

JointStates ShadowHand::get_joint_states() const
{
  return srw->joint_states_;
}

vector<Tactile> ShadowHand::get_tactiles() const
{
  return srw->tactiles_;
}

vector<string> ShadowHand::get_list_of_joints() const
{
  return srw->joint_states_.names;
}

} // namespace

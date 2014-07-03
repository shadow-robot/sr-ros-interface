#include "sr_standalone/shadow_hand.hpp"
#include "sr_standalone/sr_ros_wrapper.hpp"

using namespace std;

namespace shadow_robot_standalone
{

ShadowHand::ShadowHand()
  : wrapper_(new SrRosWrapper()) {}

ShadowHand::~ShadowHand()
{
  delete wrapper_;
}

bool ShadowHand::get_control_type(ControlType & control_type)
{
  return wrapper_->get_control_type(control_type);
}

bool ShadowHand::set_control_type(ControlType control_type)
{
  return wrapper_->set_control_type(control_type);
}

void ShadowHand::send_position(const string &joint_name, double target)
{
  wrapper_->send_position(joint_name, target);
}

void ShadowHand::send_all_positions(const vector<double> &targets)
{
  wrapper_->send_all_positions(targets);
}

void ShadowHand::send_torque(const string &joint_name, double target)
{
  wrapper_->send_torque(joint_name, target);
}

void ShadowHand::send_all_torques(const vector<double> &targets)
{
  wrapper_->send_all_torques(targets);
}

map<string, JointState> & ShadowHand::get_joint_states() const
{
  wrapper_->spin();
  return wrapper_->joint_states_;
}

vector<Tactile> & ShadowHand::get_tactiles() const
{
  wrapper_->spin();
  return wrapper_->tactiles_;
}

vector<string> ShadowHand::get_joints_with_state() const
{
  vector<string> joints_with_state;
  map<string, JointState>::const_iterator it = wrapper_->joint_states_.begin();
  while (it != wrapper_->joint_states_.end())
  {
    joints_with_state.push_back(it->first);
    ++it;
  }
  return joints_with_state;
}

vector<string> ShadowHand::get_controlled_joints() const
{
  vector<string> controlled_joints;
  boost::unordered_map<string, ros::Publisher>::const_iterator it = wrapper_->torque_pubs_.begin();
  while (it != wrapper_->torque_pubs_.end())
  {
    controlled_joints.push_back(it->first);
    ++it;
  }
  return controlled_joints;
}

} // namespace

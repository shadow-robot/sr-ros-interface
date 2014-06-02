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

// *** YI LI ***
// sr_robot_msgs/msg/ControlType.msg
// sr_self_test/src/motor_test.cpp
// void MotorTest::run_test(diagnostic_updater::DiagnosticStatusWrapper& status)
bool ShadowHand::set_control_type(ControlType control_type)
{
  wrapper_->control_type_ = control_type;
  return true;  // TODO check if control_type was actually set at the ROS side of things
}

void ShadowHand::send_position(const std::string &joint_name, double target)
{
}

void ShadowHand::send_torque(const std::string &joint_name, double target)
{
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

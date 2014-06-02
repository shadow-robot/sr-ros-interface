#include <sr_standalone/standalone.hpp>

using namespace shadow_robot_standalone;

int main(int argc, char** argv)
{
  ShadowHand hand(argc, argv);
  // JointStates joint_states = hand.get_joint_states();

  return 0;
}

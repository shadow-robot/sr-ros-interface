#include <sr_standalone/shadow_hand.hpp>

using namespace shadow_robot_standalone;

int main(int argc, char** argv)
{
  ShadowHand hand(argc, argv);
  JointStates jss = hand.get_joint_states();

  std::vector<std::string> js_names = jss.names;
  std::vector<double> positions;

  return 0;
}

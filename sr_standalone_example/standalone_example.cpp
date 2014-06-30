#include <sr_standalone/shadow_hand.hpp>
#include <iostream>
#include <time.h>

using namespace shadow_robot_standalone;
using namespace std;

int main(int argc, char** argv)
{
  ShadowHand hand(argc, argv);

  const ControlType new_ctrl_type = POSITION_PWM;
  if (hand.set_control_type(new_ctrl_type))
    cout << "Set control type to POSITION_PWM.\n";
  else
    cout << "Failed to set control type to POSITION_PWM.\n";

  ControlType curr_ctrl_type;
  if (!hand.get_control_type(curr_ctrl_type))
  {
    cout << "Failed to get control type.\n";
    return -1;
  }
  else if (curr_ctrl_type != new_ctrl_type)
  {
    cout << "Failed to set control type to POSITION_PWM.\n";
    return -1;
  }

  for (size_t counter = 0; counter < 5; ++counter)
  {
    const JointStates & jss = hand.get_joint_states();

    cout << "Joint state names:\n";
    for (size_t i = 0; i < jss.names.size(); ++i)
      cout << jss.names[i] << ", ";
    cout << "\n\n";

    cout << "Joint state positions:\n";
    for (size_t i = 0; i < jss.positions.size(); ++i)
      cout << jss.positions[i] << ", ";
    cout << "\n\n";

    cout << "List of joints:\n";
    const vector<string> & joints = hand.get_list_of_joints();
    for (size_t i = 0; i < joints.size(); ++i)
      cout << joints[i] << ", ";
    cout << "\n\n";

    cout << "Tactiles:\n";
    const vector<Tactile> & tactiles = hand.get_tactiles();
    for (size_t i = 0; i < tactiles.size(); ++i)
    {
      cout << tactiles[i].pac0 << ", "
           << tactiles[i].pac1 << ", "
           << tactiles[i].pdc << ", "
           << tactiles[i].tac << ", "
           << tactiles[i].tdc << "\n";
      for (size_t elec_i = 0; elec_i < Tactile::no_of_electrodes; ++elec_i)
        cout << tactiles[i].electrodes[elec_i] << ", ";
      cout << "\n";
    }
    cout << "\n\n";

    hand.send_position("FFJ3", 0.0);

    cout << "Sleeping...\n\n";
    sleep(1);
  }

  return 0;
}

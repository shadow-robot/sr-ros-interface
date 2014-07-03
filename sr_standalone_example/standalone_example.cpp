#include <sr_standalone/shadow_hand.hpp>
#include <iostream>
#include <unistd.h>

using namespace shadow_robot_standalone;
using namespace std;

int main(int argc, char** argv)
{
  ShadowHand hand;

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

  cout << "names of joints with states:\n";
  vector<string> joints = hand.get_joints_with_state();
  cout << joints[0];
  for (size_t i = 1; i < joints.size(); ++i)
    cout << ", " << joints[i];
  cout << "\n\n";

  cout << "names of joints that can be controlled:\n";
  vector<string> controlled_joints = hand.get_controlled_joints();
  cout << controlled_joints[0];
  for (size_t i = 1; i < controlled_joints.size(); ++i)
    cout << ", " << controlled_joints[i];
  cout << "\n\n";

  for (size_t counter = 0; counter < 5; ++counter)
  {
    map<string, JointState> & jss = hand.get_joint_states();

    for (size_t i = 0; i < joints.size(); ++i)
    {
      cout << "Joint with name : " << joints[i] << "\n"
        << "has position : " << jss[joints[i]].position << "\n"
        << "velocity : " << jss[joints[i]].velocity << "\n"
        << "and effort : " << jss[joints[i]].effort << "\n\n";
    }

    cout << "Tactiles:\n";
    vector<Tactile> & tactiles = hand.get_tactiles();
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

    cout << "Sleeping...\n\n";
    sleep(1);
  }

  hand.send_position("FFJ3", 0.5);
  sleep(1);
  hand.send_position("FFJ3", 0.0);

  return 0;
}

#include <sr_standalone/shadow_hand.hpp>
#include <iostream>
#include <boost/date_time.hpp>
#include <boost/thread.hpp>

using namespace shadow_robot_standalone;

int main(int argc, char** argv)
{
  ShadowHand hand(argc, argv);

  const ControlType new_ctrl_type = POSITION_PWM;
  if (hand.set_control_type(new_ctrl_type))
    std::cout << "Set control type to POSITION_PWM." << std::endl;
  else
    std::cout << "Failed to set control type to POSITION_PWM." << std::endl;

  std::cout << "Sleeping..." << std::endl << std::endl;
  boost::this_thread::sleep( boost::posix_time::seconds(6) );

  ControlType curr_ctrl_type;
  if (hand.get_control_type(curr_ctrl_type))
  {
    if (curr_ctrl_type != new_ctrl_type)
    {
      std::cout << "Failed to set control type to POSITION_PWM." << std::endl;
      return -1;
    }
  }
  else
  {
    std::cout << "Failed to get control type." << std::endl;
    return -1;
  }

  unsigned int counter;
  for (counter = 0; counter < 10; ++counter)
  {
    const JointStates & jss = hand.get_joint_states();

    std::cout << "Joint state names:" << std::endl;
    for (std::size_t i = 0; i < jss.names.size(); ++i)
      std::cout << jss.names[i] << ", ";
    std::cout << std::endl << std::endl;

    std::cout << "Joint state positions:" << std::endl;
    for (std::size_t i = 0; i < jss.positions.size(); ++i)
      std::cout << jss.positions[i] << ", ";
    std::cout << std::endl << std::endl;

    std::cout << "List of joints:" << std::endl;
    const std::vector<std::string> & joints = hand.get_list_of_joints();
    for (std::size_t i = 0; i < joints.size(); ++i)
      std::cout << joints[i] << ", ";
    std::cout << std::endl << std::endl;

    std::cout << "Tactiles:" << std::endl;
    const std::vector<Tactile> & tactiles = hand.get_tactiles();
    for (std::size_t i = 0; i < tactiles.size(); ++i)
    {
      std::cout << tactiles[i].pac0 << ", "
                << tactiles[i].pac1 << ", "
                << tactiles[i].pdc  << ", "
                << tactiles[i].tac  << ", "
                << tactiles[i].tdc  << std::endl;
      for( std::size_t elec_i = 0; elec_i < Tactile::no_of_electrodes; ++elec_i )
        std::cout << tactiles[i].electrodes[elec_i] << ", ";
      std::cout << std::endl;
    }
    std::cout << std::endl << std::endl;

    std::cout << "Sleeping..." << std::endl << std::endl;
    boost::this_thread::sleep( boost::posix_time::seconds(1) );
  }



  return 0;
}

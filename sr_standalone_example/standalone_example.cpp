#include <sr_standalone/shadow_hand.hpp>
#include <iostream>
#include <boost/date_time.hpp>
#include <boost/thread.hpp>

using namespace shadow_robot_standalone;

int main(int argc, char** argv)
{
  ShadowHand hand(argc, argv);

  unsigned int counter = 0;
  while (1)
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

    std::cout << "Sleeping..." << std::endl << std::endl;
    boost::posix_time::seconds dura(1);
    boost::this_thread::sleep(dura);

    if (++counter > 10)
      break;
  }

  return 0;
}

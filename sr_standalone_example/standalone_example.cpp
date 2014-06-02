#include <sr_standalone/shadow_hand.hpp>
#include <iostream>
#include <boost/date_time.hpp>
#include <boost/thread.hpp>

using namespace shadow_robot_standalone;

int main(int argc, char** argv)
{
  ShadowHand hand(argc, argv);
  const JointStates & jss = hand.get_joint_states();

  for (std::size_t counter = 0; counter < 100; ++counter)
  {
    for (std::size_t i = 0; i < jss.names.size(); ++i)
      std::cout << "Joint State Name = " << jss.names[i] << std::endl;

    for (std::size_t i = 0; i < jss.positions.size(); ++i)
      std::cout << "Joint State Positions = " << jss.positions[i] << std::endl;

    std::cout << "\nSleeping...\n";
    boost::posix_time::seconds dura(1);
    boost::this_thread::sleep(dura);
  }

  return 0;
}

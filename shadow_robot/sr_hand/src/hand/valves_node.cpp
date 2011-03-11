/**
* @file   real_arm_node.cpp
* @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
* @date   Tue Jun 29 14:56:10 2010
*
* @brief Contains the main for the real arm. We start the publishers / subscribers in this node.
* They all share the same RealArm object, this way the subscriber can update the arm properties,
* while the publishers publish up to date data. The diagnostics and the other publisher are started
* in two different threads, to allow them to be published at different frequencies.
*
*
*/

#include <ros/ros.h>

#include <boost/smart_ptr.hpp>

#include "sr_hand/hand/valves.h"

using namespace shadowrobot;

/////////////////////////////////
//           MAIN              //
/////////////////////////////////

/**
* The main function initialises this ROS subscriber and sets the different callbacks.
* This ROS subscriber will listen for new commands and send them to
* the real robot.
*
* @param argc
* @param argv
*
* @return 0 on success
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "shadowarm");
  ros::NodeHandle n;

  boost::shared_ptr<Valves> valves( new Valves() );

  while( ros::ok() )
    valves->publish();

  return 0;
}

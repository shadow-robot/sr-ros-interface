/**
* @file   real_arm_node.cpp
* @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
* @date   Tue Jun 29 14:56:10 2010
*
*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
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

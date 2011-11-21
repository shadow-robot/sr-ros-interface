/**
 * @file   etherCAT_compatibility_hand_node.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Mon Aug 22 11:17:33 2011
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
 * @brief Contains the main for the EtherCAT wrapper. We start the publishers / subscribers in this node.
 * They all share the same EtherCATCompatibilityHand object, this way the subscriber can update the hand properties,
 * while the publishers publish up to date data. The diagnostics and the other publisher are started
 * in two different threads, to allow them to be published at different frequencies.
 *
 *
 */

#include <ros/ros.h>

#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include "sr_hand/sr_subscriber.h"
#include "sr_hand/sr_publisher.h"
#include "sr_hand/sr_diagnosticer.h"
#include "sr_hand/hand/etherCAT_compatibility_hand.hpp"

using namespace std;
using namespace ros;
using namespace shadowrobot;

  /////////////////////////////////
  //           MAIN              //
  /////////////////////////////////

void run_diagnotics(boost::shared_ptr<SRDiagnosticer> shadowhand_diag)
{
  while( ok() )
  {
    shadowhand_diag->publish();
  }
}

void run_publisher(boost::shared_ptr<SRPublisher> shadowhand_pub)
{
  while( ok() )
  {
    shadowhand_pub->publish();
  }
}


/**
 * The main function initializes the links with the robot, initializes
 * this ROS subscriber and sets the different callbacks.
 * This ROS subscriber will listen for new commands and send them to
 * the robot.
 *
 * @param argc
 * @param argv
 *
 * @return 0 on success
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "shadowhand");
  NodeHandle n;

  boost::shared_ptr<EtherCATCompatibilityHand> eth_sh(new EtherCATCompatibilityHand());
  boost::shared_ptr<SRSubscriber> shadowhand_subscriber(new SRSubscriber(eth_sh));


  boost::shared_ptr<SRPublisher> shadowhand_pub( new SRPublisher(eth_sh));
  boost::shared_ptr<SRDiagnosticer> shadowhand_diag( new SRDiagnosticer(eth_sh, sr_hand_hardware));

  boost::thread thrd1( boost::bind( &run_diagnotics, shadowhand_diag ));
  boost::thread thrd2( boost::bind( &run_publisher, shadowhand_pub ));
  thrd1.join();
  thrd2.join();

  return 0;
}

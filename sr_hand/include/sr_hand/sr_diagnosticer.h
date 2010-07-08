/**
* @file   sr_diagnosticer.h
* @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
* @date   Tue May 25 17:51:10 2010
*
* @brief  The Diagnosticer is a ROS publisher which publishes diagnostic data regarding the Dextrous Hand or the Shadow
* Robot Arm, on the \/diagnostics topic. The diagnostics can be viewed using the robot_monitor package.
*
* To view the diagnostics, just rosmake robot_monitor, and then rosrun robot_monitor robot_monitor.
*
*/

#ifndef SHADOWHAND_DIAGNOSTICER_H_
#define SHADOWHAND_DIAGNOSTICER_H_

#include <ros/ros.h>
#include <boost/smart_ptr.hpp>

#include "sr_hand/hand/shadowhand.h"

using namespace ros;
using namespace shadowhand;

namespace shadowhand_diagnosticer{

/**
* The Diagnosticer is a ROS publisher which publishes diagnostic data regarding the Dextrous Hand or the Shadow Arm,
* on the \/diagnostics topic. The diagnostics can be viewed using the robot_monitor package.
*
* To view the diagnostics, just rosmake robot_monitor, and then rosrun robot_monitor robot_monitor.
*
*/

///An enum containing the different types of hardware the diagnosticer is publishing data about
enum hardware_types {
  ///the Dextrous Hand
  sr_hand_hardware,
  ///the Shadow Arm
  sr_arm_hardware
};

class ShadowhandDiagnosticer
{
public:
  /**
  * Constructor initializing the ROS node, and setting the topic to which it publishes.
  * The frequency at which this node will publish data is set by a parameter, read from ROS parameter server.
  *
  * @param sh A Shadowhand object, where the information to be published comes from.
  * @param hw_type The type of hardware we are publishing diagnostics about.
  */
  ShadowhandDiagnosticer(boost::shared_ptr<Shadowhand>  sh, hardware_types hw_type);

  /// Destructor
  ~ShadowhandDiagnosticer();

  /**
  * The callback method which is called at a given frequency. Gets the data from the shadowhand / shadowarm object.
  */
  void publish();

private:
  ///const to convert the rate data to Hz
  static const double palm_numb_msg_const;
  ///const to convert the rate data to Hz
  static const double palm_msg_rate_const;

  ///The shadowhand object (can be either an object connected to the real robot or a virtual hand).
  boost::shared_ptr<Shadowhand> shadowhand;

  ///ros node handle
  NodeHandle node, n_tilde;
  ///the rate at which the data will be published. This can be set by a parameter in the launch file.
  Rate publish_rate;

  ///The publisher which publishes the data to the \/diagnostics topic.
  Publisher shadowhand_diagnostics_pub;

  ///store the hardware_type for this diagnosticer.
  hardware_types hardware_type;
}; // end class ShadowhandDiagnosticer

} // end namespace

#endif 	    /* !SHADOWHAND_DIAGNOSTICER_H_ */

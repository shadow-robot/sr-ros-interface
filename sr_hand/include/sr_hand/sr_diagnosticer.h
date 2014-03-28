/**
 * @file   sr_diagnosticer.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Tue May 25 17:51:10 2010
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
 * @brief  The Diagnosticer is a ROS publisher which publishes diagnostic data regarding the Dextrous Hand or the Shadow
 * Robot Arm, on the \/diagnostics topic. The diagnostics can be viewed using the robot_monitor package.
 *
 * To view the diagnostics, just rosmake robot_monitor, and then rosrun robot_monitor robot_monitor.
 *
 */

#ifndef SR_DIAGNOSTICER_H_
#define SR_DIAGNOSTICER_H_

#include <ros/ros.h>
#include <boost/smart_ptr.hpp>

#include "sr_hand/hand/sr_articulated_robot.h"

using namespace ros;
using namespace shadowrobot;

namespace shadowrobot
{

/**
 * The Diagnosticer is a ROS publisher which publishes diagnostic data regarding the Dextrous Hand or the Shadow Arm,
 * on the \/diagnostics topic. The diagnostics can be viewed using the robot_monitor package.
 *
 * To view the diagnostics, just rosmake robot_monitor, and then rosrun robot_monitor robot_monitor.
 *
 */

///An enum containing the different types of hardware the diagnosticer is publishing data about
enum hardware_types
{
    ///the Dextrous Hand
    sr_hand_hardware,
    ///the Shadow Arm
    sr_arm_hardware
};

class SRDiagnosticer
{
public:
    /**
     * Constructor initializing the ROS node, and setting the topic to which it publishes.
     * The frequency at which this node will publish data is set by a parameter, read from ROS parameter server.
     *
     * @param sr_art_robot A SRArticulatedRobot object, where the information to be published comes from.
     * @param hw_type The type of hardware we are publishing diagnostics about.
     */
    SRDiagnosticer( boost::shared_ptr<SRArticulatedRobot> sr_art_robot, hardware_types hw_type );

    /// Destructor
    ~SRDiagnosticer();

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
    boost::shared_ptr<SRArticulatedRobot> sr_articulated_robot;

    ///ros node handle
    NodeHandle node, n_tilde;
    ///the rate at which the data will be published. This can be set by a parameter in the launch file.
    Rate publish_rate;

    ///The publisher which publishes the data to the \/diagnostics topic.
    Publisher sr_diagnostics_pub;

    ///store the hardware_type for this diagnosticer.
    hardware_types hardware_type;
}; // end class ShadowhandDiagnosticer

} // end namespace

#endif 	    /* !SR_DIAGNOSTICER_H_ */

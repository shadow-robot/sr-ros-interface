/**
 * @file   sr_publisher.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Apr 22 10:26:41 2010
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
 * @brief  This class reads and publishes data concerning the
 * shadowhand / shadowarm. To publish those data, just call the publish()
 * function.
 *
 *
 */

#ifndef SHADOWHAND_PUBLISHER_H_
#define SHADOWHAND_PUBLISHER_H_

#include <boost/smart_ptr.hpp>

#include <ros/ros.h>
#include "sr_hand/hand/sr_articulated_robot.h"

using namespace ros;
using namespace shadowrobot;

namespace shadowrobot
{
/**
 * This class publishes data concerning the shadowhand /shadowarm, like position, targets, forces, ... on different
 * topics. To publish those data, just call the publish() function.
 *
 * The data are published on three different topics. Two of them are joint_states topics (joint names, joint angles,
 * joint velocities, joint efforts), which are then transformed into coordinates by two different robot_state_publisher
 * (a ROS package). This is useful to visualize the data in rviz (part of ROS). A third topic is
 * \/prefix\/shadowhand_data. The messages published on this last topic are better formatted for our hardware.
 */
class SRPublisher
{
public:
    /**
     * Constructor initializing the ROS node, and setting the topic to which it publishes.
     * The frequency at which this node will publish data is set by a parameter, read from ROS parameter server.
     *
     * @param sr_art_robot A Shadowhand or Shadowarm object, where the information to be published comes from.
     */
    SRPublisher( boost::shared_ptr<SRArticulatedRobot> sr_art_robot );

    /// Destructor
    ~SRPublisher();

    /**
     * The callback method which is called at a given frequency. Gets the data from the shadowhand/shadowarm object.
     */
    void publish();

private:
    ///ros node handle
    NodeHandle node, n_tilde;
    ///the rate at which the data will be published. This can be set by a parameter in the launch file.
    Rate publish_rate;

    ///The shadowhand object (can be either an object connected to the real robot or a virtual hand).
    boost::shared_ptr<SRArticulatedRobot> sr_articulated_robot;

    ///The publisher which publishes the data to the \/{prefix}\/position\/joint_states topic.
    Publisher sr_jointstate_pos_pub;
    ///The publisher which publishes the data to the \/{prefix}\/target\/joint_states topic.
    Publisher sr_jointstate_target_pub;
    ///The publisher which publishes the data to the \/{prefix}\/shadowhand_data topic.
    Publisher sr_pub;

    /**
     * Convert an angle in degree to an angle in radians.
     * @param deg the angle in degrees
     * @return the value in rads.
     */
    inline double toRad( double deg )
    {
        return deg * 3.14159265 / 180.0;
    }
}; // end class ShadowhandPublisher

} // end namespace

#endif 	    /* !SHADOWHAND_PUBLISHER_H_ */

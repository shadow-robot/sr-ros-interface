/**
 * @file   sr_subscriber.h
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
 * @brief  This ROS subscriber is used to issue commands to the hand / arm, from sending a set of targets, to changing
 * the controller parameters.
 *
 *
 */

#ifndef SHADOWHAND_SUBSCRIBER_H_
#define SHADOWHAND_SUBSCRIBER_H_

#include <ros/ros.h>
#include <vector>
#include <string>
#include <map>

#include <boost/smart_ptr.hpp>

//messages
#include <sr_robot_msgs/joints_data.h>
#include <sr_robot_msgs/joint.h>
#include <sr_robot_msgs/contrlr.h>
#include <sr_robot_msgs/sendupdate.h>
#include <sr_robot_msgs/config.h>
#include <sr_robot_msgs/reverseKinematics.h>
#include <std_msgs/Float64.h>

#include "sr_hand/hand/sr_articulated_robot.h"

using namespace ros;
using namespace shadowrobot;

namespace shadowrobot
{

/**
 * This ROS subscriber is used to issue commands to the hand / arm, from sending a set of targets, to changing the
 * controller parameters.
 */
class SRSubscriber
{
public:
    /**
     * Constructor initializing the ROS node, and setting the topic to which it subscribes.
     *
     * @param sr_art_robot A Shadowhand object, where the information to be published comes from.
     */
    SRSubscriber( boost::shared_ptr<SRArticulatedRobot> sr_art_robot );

    /// Destructor
    ~SRSubscriber();

private:
    ///ros node handle
    NodeHandle node, n_tilde;

    ///The shadowhand / shadowarm object (can be either an object connected to the real robot or a virtual hand).
    boost::shared_ptr<SRArticulatedRobot> sr_articulated_robot;

    ///init function
    void init();

    /////////////////
    //  CALLBACKS  //
    /////////////////
    /**
     * process the sendupdate command: send new targets to the Dextrous Hand (or the Shadow Robot Arm), through the
     * shadowhand object.
     *
     * @param msg the sendupdate message received. The sendupdate message, contains the number of
     * sendupdate commands and a vector of joints with names and targets.
     */
    void sendupdateCallback( const sr_robot_msgs::sendupdateConstPtr& msg );
    ///The subscriber to the sendupdate topic.
    Subscriber sendupdate_sub;

    /**
     * Callback the etherCAT way: one topic per joint.
     *
     * @param msg the target in radians
     * @param joint_name name of the joint we're sending the command to
     */
    void cmd_callback(const std_msgs::Float64ConstPtr& msg, std::string& joint_name);
    ///The vector of subscribers to the different joint topics.
    std::vector<Subscriber> controllers_sub;

    /**
     * process the contrlr command: send new parameters to a given controller.
     * @param msg the contrlr message received. contrlr_name + list_of_parameters in a string array
     * e.g. [p:10] sets the p value of the specified controller to 10.
     */
    void contrlrCallback( const sr_robot_msgs::contrlrConstPtr& msg );
    ///The subscriber to the contrlr topic
    Subscriber contrlr_sub;

    /**
     * process the config command: send new parameters to the palm.
     * @param msg the config message received
     */
    void configCallback( const sr_robot_msgs::configConstPtr& msg );
    ///The subscriber to the config topic
    Subscriber config_sub;
}; // end class ShadowhandSubscriber

} // end namespace

#endif 	    /* !SHADOWHAND_SUBSCRIBER_H_ */

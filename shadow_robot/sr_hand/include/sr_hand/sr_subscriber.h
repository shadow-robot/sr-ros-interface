/**
 * @file   sr_subscriber.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Apr 22 10:26:41 2010
 *
 * @brief  This ROS subscriber is used to issue commands to the hand / arm, from sending a set of targets, to changing
 * the controller parameters.
 *
 *
 */

/** \example ../../examples/shadowhand_publisher.py
 * This is a python example on how to interact with this ROS subscriber to send commands to the hand.
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
     * @param sh A Shadowhand object, where the information to be published comes from.
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

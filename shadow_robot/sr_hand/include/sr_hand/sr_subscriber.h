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
#include <sr_hand/joints_data.h>
#include <sr_hand/joint.h>
#include <sr_hand/contrlr.h>
#include <sr_hand/sendupdate.h>
#include <sr_hand/config.h>

#include <tf/transform_listener.h>
//#include <tf/transform_listener.h>
#include "sr_hand/sr_kinematics.h"
#include "sr_hand/hand/sr_articulated_robot.h"

using namespace ros;
using namespace shadowrobot;
using namespace KDL;

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
    /**
     * Constructor initializing the ROS node, and setting the topic to which it subscribes.
     *
     * @param sh A Shadowhand object, where the information to be published comes from.
     * @param tree The kinematic tree of the hand. Used to compute reverse kinematics.
     */
    SRSubscriber( boost::shared_ptr<SRArticulatedRobot> sr_art_robot, KDL::Tree tree );

    /// Destructor
    ~SRSubscriber();

private:
    ///ros node handle
    NodeHandle node, n_tilde;

    ///The shadowhand / shadowarm object (can be either an object connected to the real robot or a virtual hand).
    boost::shared_ptr<SRArticulatedRobot> sr_articulated_robot;

    //contains the map for the joints of the object.
    SRArticulatedRobot::JointsMap joints_map;

    /// KDL tree containing the robot kinematic chains
    const Tree sr_kinematic_tree;
    boost::shared_ptr<SrKinematics> sr_kinematics;

    ///a vector containing the end effector names
    static const std::vector<std::string> end_effector_names;

    ///stores the current angles for the reverse kinematics
    std::vector<double> current_angles;

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
    void sendupdateCallback( const sr_hand::sendupdateConstPtr& msg );
    ///The subscriber to the sendupdate topic.
    Subscriber sendupdate_sub;

    /**
     * process the contrlr command: send new parameters to a given controller.
     * @param msg the contrlr message received. contrlr_name + list_of_parameters in a string array
     * e.g. [p:10] sets the p value of the specified controller to 10.
     */
    void contrlrCallback( const sr_hand::contrlrConstPtr& msg );
    ///The subscriber to the contrlr topic
    Subscriber contrlr_sub;

    /**
     * process the config command: send new parameters to the palm.
     * @param msg the config message received
     */
    void configCallback( const sr_hand::configConstPtr& msg );
    ///The subscriber to the config topic
    Subscriber config_sub;

    /**
     * process the reverse kinematics: uses the
     * robot_description parameter (containing the urdf description of the
     * hand) to compute the reverse kinematics.
     *
     * @todo Not yet implemented
     *
     * @param msg a tf transform message
     */
    void reverseKinematicsCallback( const tf::tfMessageConstPtr& msg );
    ///The subscriber to the reverse_kinematics topic
    Subscriber reverse_kinematics_sub;
    tf::TransformListener tf_listener;
}; // end class ShadowhandSubscriber

} // end namespace

#endif 	    /* !SHADOWHAND_SUBSCRIBER_H_ */

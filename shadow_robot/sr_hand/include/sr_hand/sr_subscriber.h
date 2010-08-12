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
#include <geometry_msgs/PoseStamped.h>

#include "sr_hand/hand/shadowhand.h"
#include "sr_hand/sr_kinematics.h"

using namespace ros;
using namespace shadowhand;

namespace shadowhand_subscriber{

/**
 * This ROS subscriber is used to issue commands to the hand / arm, from sending a set of targets, to changing the
 * controller parameters.
 */
class ShadowhandSubscriber
{
 public:
  /**
   * Constructor initializing the ROS node, and setting the topic to which it subscribes.
   *
   * @param sh A Shadowhand object, where the information to be published comes from.
   */
  ShadowhandSubscriber(boost::shared_ptr<Shadowhand> sh);
  /**
   * Constructor initializing the ROS node, and setting the topic to which it subscribes.
   *
   * @param sh A Shadowhand object, where the information to be published comes from.
   * @param tree The kinematic tree of the hand. Used to compute reverse kinematics.
   */
  ShadowhandSubscriber(boost::shared_ptr<Shadowhand>  sh, KDL::Tree tree);
  
  /// Destructor
  ~ShadowhandSubscriber();
  
 private:
  ///ros node handle
  NodeHandle node, n_tilde;

  ///The shadowhand / shadowarm object (can be either an object connected to the real robot or a virtual hand).
  boost::shared_ptr<Shadowhand>  shadowhand;

  //contains the map for the joints of the object.
  Shadowhand::JointsMap joints_map;

  boost::shared_ptr<SrKinematics> sr_kinematics;

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
  void sendupdateCallback(const sr_hand::sendupdateConstPtr& msg);
  ///The subscriber to the sendupdate topic.
  Subscriber sendupdate_sub;

  /**
   * process the contrlr command: send new parameters to a given controller.
   * @param msg the contrlr message received. contrlr_name + list_of_parameters in a string array
   * e.g. [p:10] sets the p value of the specified controller to 10.
   */
  void contrlrCallback(const sr_hand::contrlrConstPtr& msg);
  ///The subscriber to the contrlr topic
  Subscriber contrlr_sub;

  /**
   * process the config command: send new parameters to the palm.
   * @param msg the config message received
   */
  void configCallback(const sr_hand::configConstPtr& msg);
  ///The subscriber to the config topic
  Subscriber config_sub;

  /**
   * process the reverse kinematics from the given message: uses the
   * robot_description parameter (containing the urdf description of the
   * hand) to compute the reverse kinematics.
   *
   * @todo Not yet implemented
   *
   * @param msg the reverse kinematic message
   */
  void reverseKinematicsCallback( const geometry_msgs::PoseStampedConstPtr& msg );
  ///The subscriber to the reverse_kinematics topic
  Subscriber reverse_kinematics_sub;

}; // end class ShadowhandSubscriber

} // end namespace

#endif 	    /* !SHADOWHAND_SUBSCRIBER_H_ */

/**
 * @file   shadowhand_to_cyberglove_remapper.h
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu May 13 09:44:52 2010
 *
 * @brief This program remaps the force information contained in
 * /joint_states coming from the hand to the /cybergraspforces topic
 * used to control the cybergrasp.
 *
 *
 */

#ifndef   	SHADOWHAND_TO_CYBERGLOVE_REMAPPER_H_
# define   	SHADOWHAND_TO_CYBERGLOVE_REMAPPER_H_

//messages
#include <sensor_msgs/JointState.h>
#include "sr_remappers/calibration_parser.h"

using namespace ros;

namespace shadowhand_to_cyberglove_remapper{

/**
 * This program remaps the force information contained in
 * /joint_states coming from the hand to the /cybergraspforces topic
 * used to control the cybergrasp.
 */
class ShadowhandToCybergloveRemapper
{
 public:
  /**
   * Init the publisher / subscriber, the joint names, read the calibratin matrix
   */
  ShadowhandToCybergloveRemapper();
  ~ShadowhandToCybergloveRemapper(){};
 private:
  /**
   * Number of joints in the hand
   */
  static const int number_hand_joints;

  /**
   * Init the vector containing the joints names
   *
   */
  void init_names();
  /// ROS node handles
  NodeHandle node, n_tilde;
  /// Vector containing all the joints names for the shadowhand.
  std::vector<std::string> joints_names; 
  /// subscriber to the jointstates topic from the cyberglove
  Subscriber cyberglove_jointstates_sub;
  ///publish to the shadowhand sendupdate topic
  Publisher shadowhand_pub;
  ///the calibration parser containing the mapping matrix
  CalibrationParser* calibration_parser;

  /////////////////
  //  CALLBACKS  //
  /////////////////

  /**
   * process the joint_states callback: receives the message from the cyberglove node, remap it to the Dextrous hand and
   * publish this message on a given topic
   *
   * @param msg the joint_states message
   */
  void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg);
   
}; // end class

} //end namespace

#endif 	    /* !SHADOWHAND_TO_CYBERGLOVE_REMAPPER_H_ */

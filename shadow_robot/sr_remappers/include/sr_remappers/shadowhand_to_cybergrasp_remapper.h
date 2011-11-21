/**
 * @file   shadowhand_to_cybergrasp_remapper.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu May 13 09:44:52 2010
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
 * @brief This program remapps the force information contained in
 * /joint_states coming from the hand to the /cybergraspforces topic
 * used to control the cybergrasp.
 *
 *
 */

#ifndef   	SHADOWHAND_TO_CYBERGRASP_REMAPPER_H_
# define   	SHADOWHAND_TO_CYBERGRASP_REMAPPER_H_

//messages
#include <sensor_msgs/JointState.h>
#include <cybergrasp/cybergraspforces.h>

#include "sr_remappers/calibration_parser.h"

using namespace ros;

namespace shadowhand_to_cybergrasp_remapper{

/**
 * @brief This program remaps the force information contained in
 * /joint_states coming from the hand to the /cybergraspforces topic
 * used to control the cybergrasp.
 *
 */
class ShadowhandToCybergraspRemapper
{
 public:
  /**
   * Init the publisher / subscriber, the joint names, read the calibratin matrix
   */
  ShadowhandToCybergraspRemapper();
  ~ShadowhandToCybergraspRemapper();

 private:
  /// ROS node handles
  NodeHandle node, n_tilde;

  ///subscribe to the shadowhand sendupdate topic
  Subscriber shadowhand_jointstates_sub;
  ///publish to the cybergrasp /cybergraspforces topic
  Publisher shadowhand_cybergrasp_pub;
  ///the calibration parser containing the mapping matrix
  CalibrationParser* calibration_parser;
  
  /////////////////
  //  CALLBACKS  //
  /////////////////

  /**
   * process the joint_states callback: receives the message from the shadowhand node, remap it to the Cybergrasp and
   * publish this message on a given topic
   *
   * @param msg the joint_states message
   */
  void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg);
  
}; // end class

} //end namespace

#endif 	    /* !CYBERGRASP_TO_SHADOWHAND_REMAPPER_H_ */

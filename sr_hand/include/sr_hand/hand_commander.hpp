/**
 * @file   hand_commander.hpp
 * @author Toni Oliver <toni@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Thu Nov 08 15:34:37 2012
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
 * @brief  This is a library that offers a simple interface to send commands to hand joints.
 * It is compatible with the Shadow Robot CAN hand and ethercat hand.
 * It allows the user not worry about the name of the currently running controllers.
 * Only position control is allowed (targets must represent angles).
 *
 *
 */

#pragma once

#include <ros/ros.h>
#include <vector>
#include <string>
#include <map>

#include <boost/smart_ptr.hpp>
#include <boost/ptr_container/ptr_map.hpp>
#include <sr_robot_msgs/joint.h>
#include <urdf/model.h>

using namespace ros;

namespace shadowhandRosLib
{
  enum HandType
  {
    UNKNOWN,
    CAN,
    ETHERCAT
  };
}

namespace shadowrobot
{
/**
 * This ROS subscriber is used to issue commands to the hand / arm, from sending a set of targets, to changing the
 * controller parameters.
 */
  class HandCommander
  {
  public:
    HandCommander(const std::string& ns = "");

    /// Destructor
    ~HandCommander();

    void sendCommands(std::vector<sr_robot_msgs::joint> joint_vector);

    /**
     * Returns the topic name for the controller state of a given joint. Useful for
     * easily subscribing to the corresponding state topic in another node.
     *
     * @param joint_name the joint for which you want the topic.
     *
     * @return the full name (fully resolved) of the topic to which you need to subscribe.
     */
    std::string get_controller_state_topic(std::string joint_name);

    /**
     * Reads the min and max for a given joint from the urdf description.
     *
     * @param joint_name name of the joint (upper or lower case) (e.g. FFJ3)
     *
     * @return a pair containing first the min then the max for the given joint.
     */
    std::pair<double, double> get_min_max(std::string joint_name);

    /**
     * Get all the joint names from the robot description.
     *  Ideal for looping over all joints.
     *
     * @return A vector of joint names.
     */
    std::vector<std::string> get_all_joints();

  private:
    ///ros node handle
    NodeHandle node_;

    ///stores data about the hand (read from urdf)
    std::map<std::string, boost::shared_ptr<urdf::Joint> > all_joints;

    ///Publisher for the CAN hand targets
    Publisher sr_hand_target_pub;
    ///Publishers for the ethercat hand targets for every joint
    boost::ptr_map<std::string,Publisher> sr_hand_target_pub_map;

    ///A map of topics for the controller states
    std::map<std::string, std::string> sr_hand_sub_topics;

    shadowhandRosLib::HandType hand_type;
    bool ethercat_controllers_found;

    /**
     * init function for the ethercat hand
     * It can be called if we know that there's an ethercat hand (controller_manager running)
     */
    void initializeEthercatHand();

    static const double TIMEOUT_TO_DETECT_CONTROLLER_MANAGER;
  }; // end class ShadowhandSubscriber

} // end namespace

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

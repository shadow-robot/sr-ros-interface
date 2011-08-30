/**
 * @file   etherCAT_compatibility_hand.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Mon Aug 22 10:33:35 2011
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
 * @brief  This compatibility interface is a wrapper around the new etherCAT Hand
 * ROS driver. It is used to present the same interface as the CAN hand.
 *
 * Please be aware that if you're developing Software for the EtherCAT hand only,
 * you should probably use the etherCAT ROS interface directly.
 *
 *
 */

#ifndef   	_ETHERCAT_COMPATIBILITY_HAND_HPP_
# define   	_ETHERCAT_COMPATIBILITY_HAND_HPP_

#include "sr_hand/hand/sr_articulated_robot.h"
#include <sensor_msgs/JointState.h>

namespace shadowrobot
{
/**
 * @brief This compatibility interface is a wrapper around the new etherCAT Hand
 * ROS driver. It is used to present the same interface as the CAN hand.
 *
 */
  class EtherCATCompatibilityHand : public SRArticulatedRobot
  {
  public:
    /**
     * Initializes the necessary mappings with a static list of names.
     */
    ROS_DEPRECATED EtherCATCompatibilityHand();
    ///destructor
    ~EtherCATCompatibilityHand();

    //virtual classes defined in Shadowhand
    /**
     * This function will send the targets to the correct controllers.
     *
     * @param joint_name The Joint in joints_map you wish to send the target to.
     * @param target The target in degree
     * @return 0 if success ; -1 if error
     */
    virtual short sendupdate( std::string joint_name, double target );

    /**
     * Returns the last data we received for the given joint.
     *
     * @param joint_name The name of the joint, as specified in joints_map.
     * @return The information regarding this joint.
     */
    virtual JointData getJointData( std::string joint_name );
    virtual JointsMap getAllJointsData();

    virtual short setContrl( std::string contrlr_name, JointControllerData ctrlr_data );
    virtual JointControllerData getContrl( std::string ctrlr_name );

    virtual short setConfig( std::vector<std::string> myConfig );
    virtual void getConfig( std::string joint_name );

    /**
     * Not used in this interface: the diagnostics are published directly by the EtherCAT hand
     * driver.
     *
     *@return A vector containing all the diagnostics for the hand (motor information, etc...)
     */
    virtual std::vector<DiagnosticData> getDiagnostics();
  protected:
    ros::NodeHandle node, n_tilde;

    /**
     * This callback is called each time a joint state message is received. We
     * Update the internal joint map when we receive this message.
     *
     * @param msg the joint state message.
     */
    void joint_states_callback(const sensor_msgs::JointStateConstPtr& msg);

    /**
     * Initialize a mapping for the joints and the publishers.
     */
    void initializeMap();

    ///This vector stores publishers to each joint controller.
    std::vector< ros::Publisher > etherCAT_publishers;

    ///a subscriber for the /joint_states topic.
    ros::Subscriber joint_state_subscriber;
  }; //end class
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/


#endif 	    /* !_ETHERCAT_COMPATIBILITY_HAND_HPP_ */

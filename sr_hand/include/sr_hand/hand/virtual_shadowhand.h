/**
 * @file   virtual_shadowhand.h
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
 * @brief  The Virtual Shadowhand can be used as a simulator. As both the real hand and the virtual hand are children
 * from the shadowhand class, using a virtual or a real hand doesn't change anything in the way you call them in your
 * programs.
 *
 *
 */

#ifndef   	VIRTUAL_SHADOWHAND_H_
# define   	VIRTUAL_SHADOWHAND_H_

#include "sr_hand/hand/sr_articulated_robot.h"

namespace shadowrobot
{
/**
 * @brief  The Virtual Shadowhand can be used as a simulator. As both the real hand and the virtual hand are children
 * from the shadowhand class, using a virtual or a real hand doesn't change anything in the way you call them in your
 * programs.
 *
 */

class VirtualShadowhand : public SRArticulatedRobot
{
public:
    /**
     * Initializes the necessary mappings with a static list of names.
     */
    VirtualShadowhand();
    virtual ~VirtualShadowhand();

    //virtual classes defined in Shadowhand
    /**
     * This function will set the target of the object to the given angle. It will also set the position to this
     * target.
     *
     * @todo This could be improved by implementing a control algorithm in this theoretic hand.
     *
     * @param joint_name The Joint in joints_map you wish to send the target to.
     * @param target The target in degree
     * @return 0 if success ; -1 if error
     */
    virtual short sendupdate( std::string joint_name, double target );

    /**
     * In the virtual hand, getJointData() simply fetches the data from a given joint in the joints_map. As the targets
     * and positions are up to date, there's no need of reading other values here for those two values. However, we
     * generate random values for the other data (temperature, current, etc...)
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
     * Generates a set of random data to be published by the diagnostic publisher, but keep the position and target as
     * they are ( those are updated by the sendupdate() function).
     * @return A vector containing all the diagnostics for the hand (motor information, etc...)
     */
    virtual std::vector<DiagnosticData> getDiagnostics();
protected:
    ///ROS Node handles
    ros::NodeHandle node, n_tilde;

#ifdef GAZEBO
    /**
     * Updates the internal map with the current positions / etc...
     * from Gazebo.
     *
     * @param msg the joint_state message coming from Gazebo
     */
    void gazeboCallback(const sensor_msgs::JointStateConstPtr& msg);
#endif

    /**
     * Initialize a mapping for the joints as well as a mapping for the controllers.
     */
    void initializeMap();

    ///@see controllers_map
    typedef std::map<std::string, JointControllerData> ControllersMap;
    ///Contains the mapping between the controller names and their data.
    ControllersMap controllers_map;

    /**
     * Convert an angle in degree to an angle in radians.
     * @param deg the angle in degrees
     * @return the value in rads.
     */
    inline double toRad( double deg )
    {
        return deg * 3.14159265 / 180.0;
    }

    /**
     * Convert an angle in radian to an angle in degrees.
     * @param rad the angle in rads
     * @return the value in degrees.
     */
    inline double toDegrees( double rad )
    {
        return rad * 180.0 / 3.14159265;
    }
}; //end class
}
#endif 	    /* !VIRTUAL_SHADOWHAND_H_ */

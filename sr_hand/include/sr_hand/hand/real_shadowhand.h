/**
 * @file   real_shadowhand.h
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
 * @brief The real shadowhand is the ROS interface to Shadow Robot robotic hand.
 * 
 * @todo Make sure it works with the motor hand.
 * 
 */

#ifndef   	REAL_SHADOWHAND_H_
# define   	REAL_SHADOWHAND_H_

#include <robot/config.h>
#include "sr_hand/hand/sr_articulated_robot.h"

namespace shadowrobot
{
/**
 * The real shadowhand class is a class used to access the C code of the Dextrous Hand.
 */
class RealShadowhand : public SRArticulatedRobot
{
public:
    /**
     * Constructor for the real Shadowhand: initialize the connection to the robot and also initialize the
     * joint_map containing the mapping between joint_names and their data.
     */
    RealShadowhand();

    ///Destructor
    ~RealShadowhand();

    /**
     * Send a new target to a given joint on the robot. The command will be issued to the robot which will move if
     * the controllers are started and tuned. The target is truncated to the correct range.
     *
     * @param joint_name The Joint in joints_map you wish to send the target to.
     * @param target The target in degree
     * @return 0 if success ; -1 if error
     */
    virtual short sendupdate( std::string joint_name, double target );

    virtual JointData getJointData( std::string joint_name );
    virtual JointsMap getAllJointsData();

    virtual short setContrl( std::string contrlr_name, JointControllerData ctrlr_data );
    virtual JointControllerData getContrl( std::string contrlr_name );
    virtual short setConfig( std::vector<std::string> myConfig );
    virtual void getConfig( std::string joint_name );
    virtual std::vector<DiagnosticData> getDiagnostics();
protected:
    /***
     * Initializes the mapping between the joint_names and their data. This function fetches the joint_names from
     * the robot code.
     */
    void initializeMap();
}; //end class
}
#endif 	    /* !REAL_SHADOWHAND_H_ */

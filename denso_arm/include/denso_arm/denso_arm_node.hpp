/**
 * @file   denso_arm_node.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Mon Oct 31 09:26:15 2011
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
 * @brief A ROS interface to the DENSO arm.
 *
 *
 */

#ifndef _DENSO_ARM_NODE_HPP_
#define _DENSO_ARM_NODE_HPP_

#include <ros/ros.h>
#include <boost/smart_ptr.hpp>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

#include <denso_msgs/MoveArmPoseAction.h>
#include <actionlib/server/simple_action_server.h>

#include "denso_arm/denso_arm.hpp"
#include "denso_arm/denso_joints.hpp"

namespace denso
{
  class DensoArmNode
  {
  public:
    DensoArmNode();
    virtual ~DensoArmNode();

    void update_joint_states_callback(const ros::TimerEvent& e);

    typedef actionlib::SimpleActionServer<denso_msgs::MoveArmPoseAction> MoveArmPoseServer;

    void go_to_arm_pose(const denso_msgs::MoveArmPoseGoalConstPtr& goal);

  protected:
    ros::NodeHandle node_;
    boost::shared_ptr<DensoJointsVector> denso_joints_;

    boost::shared_ptr<DensoArm> denso_arm_;

    ////////
    //Publishing joint states
    /// A timer to publish the joint states at a given frequency.
    ros::Timer timer_joint_states_;
    /// The publisher used to publish joint states for the arm.
    ros::Publisher publisher_js_;
    ///The message published
    sensor_msgs::JointState joint_state_msg_;

    ///////
    //Action lib server for moving the arm in cartesian space.
    ///An action server for the MoveArmPose actions.
    boost::shared_ptr<MoveArmPoseServer> move_arm_pose_server_;

    ///Initialize the denso_joints_ vector.
    void init_joints();
  };
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif

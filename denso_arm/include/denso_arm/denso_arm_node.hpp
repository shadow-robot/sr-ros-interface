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

#include <math.h>

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

    ///The result to a MoveArmPoseGoal
    denso_msgs::MoveArmPoseResult move_arm_pose_result_;
    ///The feedback: published while waiting for the action to be finished
    denso_msgs::MoveArmPoseFeedback move_arm_pose_feedback_;

    ///Initialize the denso_joints_ vector.
    void init_joints();

    /**
     * Transform an incoming geometry_msgs::Pose message
     * to a pose for the denso arm.
     *
     * @param geom_pose the incoming pose demand
     *
     * @return the pose for the denzo arm.
     */
    Pose geometry_pose_to_denso_pose(geometry_msgs::Pose geom_pose)
    {
      Pose pose_denso;
      pose_denso.x = geom_pose.position.x;
      pose_denso.y = geom_pose.position.y;
      pose_denso.z = geom_pose.position.z;

      //compute rpy from quaternion (cf http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)
      pose_denso.roll = atan( (2*(geom_pose.orientation.x*geom_pose.orientation.y + geom_pose.orientation.z*geom_pose.orientation.w))
                              / ( 1 - 2*(geom_pose.orientation.y*geom_pose.orientation.y + geom_pose.orientation.z*geom_pose.orientation.z))  );
      pose_denso.pitch = asin( 2*(geom_pose.orientation.x*geom_pose.orientation.z - geom_pose.orientation.w*geom_pose.orientation.y) );
      pose_denso.yaw = atan( (2*(geom_pose.orientation.x*geom_pose.orientation.w + geom_pose.orientation.y*geom_pose.orientation.z))
                             / ( 1 - 2*(geom_pose.orientation.z*geom_pose.orientation.z + geom_pose.orientation.w*geom_pose.orientation.w))  );

      return pose_denso;
    };

  };
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif

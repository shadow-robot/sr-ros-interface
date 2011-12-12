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
#include <denso_msgs/SetTooltip.h>
#include <actionlib/server/simple_action_server.h>
#include <tf/transform_broadcaster.h>
#include <sr_utilities/sr_math_utils.hpp>

#include "denso_arm/denso_arm.hpp"
#include "denso_arm/denso_joints.hpp"

#include <boost/thread.hpp>

#include <math.h>

namespace denso
{
  class DensoArmNode
  {
  public:
    DensoArmNode();
    virtual ~DensoArmNode();

    void update_joint_states_callback(const ros::TimerEvent& e);
    void update_tip_pose_callback(const ros::TimerEvent& e);

    typedef actionlib::SimpleActionServer<denso_msgs::MoveArmPoseAction> MoveArmPoseServer;

    void go_to_arm_pose(const denso_msgs::MoveArmPoseGoalConstPtr& goal);

    bool set_tooltip( denso_msgs::SetTooltip::Request& req,
                      denso_msgs::SetTooltip::Response& resp );

  protected:
    ros::NodeHandle node_;
    /// a vector containing the joint positions for the denso arm
    boost::shared_ptr<DensoJointsVector> denso_joints_;

    ///a pointer to the denso arm driver
    boost::shared_ptr<DensoArm> denso_arm_;

    ///the cartesian pose of the tip of the arm.
    boost::shared_ptr<Pose> tip_pose_;

    ////////
    //Publishing joint states
    /// A timer to publish the joint states at a given frequency.
    ros::Timer timer_joint_states_;
    /// The publisher used to publish joint states for the arm.
    ros::Publisher publisher_js_;
    ///The message published
    sensor_msgs::JointState joint_state_msg_;

    ////////
    //Publishing tip pose
    /// A timer to publish the cartesian pose of the tip of the arm at a given frequency.
    ros::Timer timer_tip_pose_;
    /// A transform broadcaster used to publish the tip pose
    boost::shared_ptr<tf::TransformBroadcaster> tf_tip_broadcaster_;

    ///////
    //Action lib server for moving the arm in cartesian space.
    ///An action server for the MoveArmPose actions.
    boost::shared_ptr<MoveArmPoseServer> move_arm_pose_server_;

    ///The result to a MoveArmPoseGoal
    denso_msgs::MoveArmPoseResult move_arm_pose_result_;
    ///The feedback: published while waiting for the action to be finished
    denso_msgs::MoveArmPoseFeedback move_arm_pose_feedback_;

    ///A service server to be able to specify the tooltip pose (for IK)
    ros::ServiceServer tooltip_server;

    ///Initialize the denso_joints_ vector.
    void init_joints();

    /**
     * mutex to make sure we don't send two commands at the same time
     *  to the denso arm
     */
    boost::mutex denso_mutex;

    ///Controls whether we're instantiating the driver or not.
    static const bool simulated;

    /**
     * Transform an incoming geometry_msgs::Pose message
     * to a pose for the denso arm.
     *
     * @param geom_pose the incoming pose demand
     *
     * @return the pose for the denzo arm.
     */
    Pose geometry_pose_to_denso_pose(const geometry_msgs::Pose& geom_pose)
    {
      Pose pose_denso;
      //the position needs to be sent in mm to the denso arm
      pose_denso.x = geom_pose.position.x * 1000.0;
      pose_denso.y = geom_pose.position.y * 1000.0;
      pose_denso.z = geom_pose.position.z * 1000.0;

      double roll, pitch, yaw;
      btQuaternion q;
      tf::quaternionMsgToTF(geom_pose.orientation, q);
      btMatrix3x3(q).getRPY(roll, pitch, yaw);
      //compute rpy from quaternion (cf http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)
      // The rpy needs to be sent in degrees to the denso arm
      pose_denso.roll = sr_math_utils::to_degrees(roll);
      pose_denso.pitch = sr_math_utils::to_degrees(pitch);
      pose_denso.yaw = sr_math_utils::to_degrees(yaw);

      return pose_denso;
    };

    /**
     * Transforms a denso Pose to a tf::Transform.
     *
     * @param pose The pose we want to transform.
     *
     * @return The equivalent pose as a tf.
     */
    tf::Transform denso_pose_to_tf_transform(boost::shared_ptr<Pose> pose)
    {
      tf::Transform tf;
      tf::Quaternion quat;

      quat.setRPY( sr_math_utils::to_rad(pose->roll), sr_math_utils::to_rad(pose->pitch), sr_math_utils::to_rad(pose->yaw) );
      tf.setOrigin( tf::Vector3(pose->x / 1000.0, pose->y / 1000.0 , pose->z / 1000.0) );
      tf.setRotation( quat );

      return tf;
    }

  };
}


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif

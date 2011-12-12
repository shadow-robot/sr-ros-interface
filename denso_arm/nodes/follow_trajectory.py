#!/usr/bin/env python
#
# Copyright 2011 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#

import roslib; roslib.load_manifest('denso_arm')
import rospy

import actionlib
from denso_msgs.msg import TrajectoryGoal, TrajectoryResult, TrajectoryFeedback, TrajectoryAction
from denso_msgs.msg import MoveArmPoseGoal, MoveArmPoseResult, MoveArmPoseAction
from geometry_msgs.msg import Pose
import tf
from math import acos, cos, sin, sqrt

DEFAULT_RATE = 50.
DEFAULT_TIMEOUT = 5.

class DensoTrajectoryFollower( object ):
    """
    Follows a trajectory (list of poses) with the denso arm.
    """

    def __init__(self):
        rospy.init_node("follow_trajectory")

        # a transform listener to be able to get the current pose for the arm
        self.tf_listener = tf.TransformListener()

        #actionlib client to the denso arm
        self.denso_arm_client = actionlib.SimpleActionClient( '/denso_arm/move_arm_pose', MoveArmPoseAction )
        self.denso_arm_client.wait_for_server()

        #actionlib server for getting the instructions
        self.trajectory_server = actionlib.SimpleActionServer( '/denso_arm/trajectory', TrajectoryAction,
                                                               execute_cb = self.trajectory_cb )

        self.trajectory_result = TrajectoryResult()
        self.trajectory_feedback = TrajectoryFeedback()

        self.trajectory_server.start()

        rospy.spin()

    def trajectory_cb(self, goal):
        rospy.loginfo( "Got a Trajectory request.")
        if len(goal.speed) != len(goal.trajectory):
            rospy.logerr("Wrong size for the speed vector.")
            return 

        success = TrajectoryResult.SUCCESS
        start_time = rospy.Time.now()
        total_available_time = rospy.Duration.from_sec(len(goal.trajectory) * DEFAULT_TIMEOUT)
        for step_index, pose in enumerate(goal.trajectory):
            if self.trajectory_server.is_preempt_requested():
                rospy.loginfo("Denso Trajectory preempted.")
                success = TrajectoryResult.PREEMPTED
                break

            #publish feedback
            self.trajectory_feedback.step_index = step_index
            #compute the time we have left
            self.trajectory_feedback.total_time_left = total_available_time - (rospy.Time.now() - start_time)

            #interpolate between where we are and where we want to go
            # otherwise the IK on the arm fails
            pose_tip = self.get_pose("/denso_arm/tooltip")
            interpolation_steps = 100
            for i in range( 0, interpolation_steps ):
                pose_tmp = Pose()

                pose_tmp.position.x = pose_tip.position.x + (pose.position.x - pose_tip.position.x) * float(i) / float(interpolation_steps)
                pose_tmp.position.y = pose_tip.position.y + (pose.position.y - pose_tip.position.y) * float(i) / float(interpolation_steps)
                pose_tmp.position.z = pose_tip.position.z + (pose.position.z - pose_tip.position.z) * float(i) / float(interpolation_steps)

                pose_tmp.orientation = self.slerp( pose_tip.orientation, pose.orientation, float(i) / float(interpolation_steps) )

                tmp_goal = MoveArmPoseGoal()
                tmp_goal.goal = pose_tmp
                tmp_goal.rate = DEFAULT_RATE
                tmp_goal.time_out = rospy.Duration.from_sec( DEFAULT_TIMEOUT )
                tmp_goal.speed = goal.speed[step_index]

                self.denso_arm_client.send_goal( tmp_goal )
                self.denso_arm_client.wait_for_result()

                res =  self.denso_arm_client.get_result()
                if res.val != MoveArmPoseResult.SUCCESS:
                    rospy.logwarn("Failed to move the arm to the given position.")
                    success = TrajectoryResult.FAILED
                    break

            if success != TrajectoryResult.SUCCESS:
                break

        self.trajectory_result.val = success
        if success == TrajectoryResult.SUCCESS:
            self.trajectory_server.set_succeeded( self.trajectory_result )
        elif success == TrajectoryResult.PREEMPTED:
            self.trajectory_server.set_preempted( success )
        else:
            self.trajectory_server.set_aborted( success )

    def slerp(self, qa, qb, t):
        qm = qa
        cos_half_theta = qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z;
	# if qa=qb or qa=-qb then theta = 0 and we can return qa
	if (abs(cos_half_theta) >= 1.0):
            qm.w = qa.w
            qm.x = qa.x
            qm.y = qa.y
            qm.z = qa.z
            return qm
        # Calculate temporary values.
	halfTheta = acos(cos_half_theta)
	sinHalfTheta = sqrt(1.0 - cos_half_theta*cos_half_theta);
	# if theta = 180 degrees then result is not fully defined
	# we could rotate around any axis normal to qa or qb
	if (abs(sinHalfTheta) < 0.001):
            qm.w = (qa.w * 0.5 + qb.w * 0.5)
            qm.x = (qa.x * 0.5 + qb.x * 0.5)
            qm.y = (qa.y * 0.5 + qb.y * 0.5)
            qm.z = (qa.z * 0.5 + qb.z * 0.5)
            return qm

	ratioA = sin((1 - t) * halfTheta) / sinHalfTheta
	ratioB = sin(t * halfTheta) / sinHalfTheta
	#calculate Quaternion.
	qm.w = (qa.w * ratioA + qb.w * ratioB)
	qm.x = (qa.x * ratioA + qb.x * ratioB)
	qm.y = (qa.y * ratioA + qb.y * ratioB)
	qm.z = (qa.z * ratioA + qb.z * ratioB)
	return qm

    def get_pose(self, link_name):
        trans = None
        rot = None

        #try to get the pose of the given link_name
        # in the base_link frame.
        for i in range (0, 500):
            try:
                (trans, rot) = self.tf_listener.lookupTransform( '/base_link', link_name,
                                                                 rospy.Time(0) )
                break
            except (tf.LookupException, tf.ConnectivityException):
                continue

        pose = None
        pose = Pose()
        pose.position.x = trans[0]
        pose.position.y = trans[1]
        pose.position.z = trans[2]
        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]

        return pose


if __name__ == '__main__':
    denso_traj = DensoTrajectoryFollower()

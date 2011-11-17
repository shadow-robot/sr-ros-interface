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

DEFAULT_RATE = 50.
DEFAULT_TIMEOUT = 5.

class DensoTrajectoryFollower( object ):
    """
    Follows a trajectory (list of poses) with the denso arm.
    """

    def __init__(self):
        rospy.init_node("follow_trajectory")

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

        success = TrajectoryResult.SUCCESS
        start_time = rospy.Time.now()
        total_available_time = len(goal.trajectory) * rospy.Duration.from_sec(DEFAULT_TIMEOUT)
        for step_index, pose in enumerate(goal.trajectory):
            if self.trajectory_server.is_preempt_requested():
                rospy.loginfo("Denso Trajectory preempted.")
                success = TrajectoryResult.PREEMPTED
                break
            arm_goal = MoveArmPoseGoal()
            arm_goal.goal = pose
            arm_goal.rate = DEFAULT_RATE
            #compute the time we have left
            arm_goal.time_out = total_available_time - (rospy.Time.now() - start_time)

            #publish feedback
            self.trajectory_feedback.step_index = step_index
            self.trajectory_feedback.total_time_left =

            self.denso_arm_client.send_goal( arm_goal )
            self.denso_arm_client.wait_for_result()

            res =  self.denso_arm_client.get_result()
            if res.val != MoveArmPoseResult.SUCCESS:
                rospy.logwarn("Failed to move the arm to the given position.")
                success = TrajectoryResult.FAILED
                break

        self.trajectory_result.val = success
        if success == TrajectoryResult.SUCCESS:
            self.trajectory_server.set_succeeded( self.trajectory_result )
        elif success == TrajectoryResult.PREEMPTED:
            self.trajectory_server.set_preempted( success )
        else:
            self.trajectory_server.set_aborted( success )

if __name__ == '__main__':
    denso_traj = DensoTrajectoryFollower()

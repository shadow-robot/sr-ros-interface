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
        self.denso_arm_client = actionlib.SimpleActionClient('/denso_arm/move_arm_pose', MoveArmPoseAction)
        self.denso_arm_client.wait_for_server()

        #actionlib server for getting the instructions
        #TODO

        rospy.spin()

    def follow_trajectory(self, list_of_poses):
        for pose in list_of_poses:
            arm_goal = MoveArmPoseGoal()
            arm_goal.goal = pose
            arm_goal.rate = DEFAULT_RATE
            arm_goal.time_out = rospy.Duration.from_sec(DEFAULT_TIMEOUT)

            self.denso_arm_client.send_goal( arm_goal )
            self.denso_arm_client.wait_for_result()

            res =  self.denso_arm_client.get_result()
            if res.val != MoveArmPoseResult.SUCCESS:
                rospy.logerr("Failed to move the arm to the given position.")
                return False

        return True

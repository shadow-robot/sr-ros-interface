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

import denso_msgs.msg
from geometry_msgs.msg import Pose

def denso_arm_client( rate, time_out ):
    client = actionlib.SimpleActionClient('/denso_arm/move_arm_pose', denso_msgs.msg.MoveArmPoseAction)
    client.wait_for_server()

    pose_goal = Pose()
    pose_goal.position.x = 0.52013
    pose_goal.position.y = 0.06585
    pose_goal.position.z = 0.28740

    pose_goal.orientation.x = 0.0
    pose_goal.orientation.y = 0.7071067811865476
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = 0.7071067811865476

    goal = denso_msgs.msg.MoveArmPoseGoal(pose_goal, rate, rospy.rostime.Duration(time_out) )

    client.send_goal(goal)

    client.wait_for_result()

    print client.get_result()

if __name__ == '__main__':
    rospy.init_node('test_denso_arm_node', anonymous=True)

    denso_arm_client( 50, 40)


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

def build_pose(x, y, z):
    pose_goal = Pose()
    pose_goal.position.x = x
    pose_goal.position.y = y
    pose_goal.position.z = z

    pose_goal.orientation.x = 0.0
    pose_goal.orientation.y = 0.7071067811865476
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = 0.7071067811865476

    return pose_goal

def send_pose( x, y, z ):
    rate = 50
    time_out = 10
    client = actionlib.SimpleActionClient('/denso_arm/move_arm_pose', denso_msgs.msg.MoveArmPoseAction)
    client.wait_for_server()

    pose_goal = build_pose( x, y, z )
    goal = denso_msgs.msg.MoveArmPoseGoal(pose_goal, rate, rospy.rostime.Duration(time_out), 10. )

    client.send_goal(goal)

    client.wait_for_result()

    print client.get_result()

def send_trajectory( nb_poses ):
    client = actionlib.SimpleActionClient('/denso_arm/trajectory', denso_msgs.msg.TrajectoryAction)
    client.wait_for_server()

    goal = denso_msgs.msg.TrajectoryGoal

    start_x = 0.52013
    start_y = 0.06585
    start_z = 0.28740

    traj = []
    speeds = []
    for i in range(0, nb_poses ):
        traj.append( build_pose(start_x + i*0.05, start_y, start_z ) )
        speeds.append( 10. )

    goal.trajectory = traj

    client.send_goal( goal )
    client.wait_for_result()

    print client.get_result()


if __name__ == '__main__':
    rospy.init_node('test_denso_arm_node', anonymous=True)

    #test sending a pose directly
    send_pose( 0.52013, 0.06585, 0.28740 )

    #test sending a trajectory
    #send_trajectory( 5 )

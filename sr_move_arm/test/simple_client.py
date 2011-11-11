#!/usr/bin/env python

import roslib; roslib.load_manifest("sr_move_arm")
import rospy
import actionlib

from arm_navigation_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node('sr_move_arm_client')
    client = actionlib.SimpleActionClient('/right_arm/move_arm', MoveArmAction)
    client.wait_for_server()

    goal = MoveArmGoal()
    client.send_goal(goal)
    client.wait_for_result()
    print client

#!/usr/bin/env python

from sys import argv

import rospy
from moveit_msgs.srv import SaveRobotStateToWarehouse as SaveState
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState


class GraspSaver:
    def __init__(self, name):
        self.__group_name = "right_hand"
        self.__save = rospy.ServiceProxy(
            '/moveit_warehouse_services/save_robot_state', SaveState)
        self.__js_subscriber = rospy.Subscriber("joint_states",
                                                JointState,
                                                self.__js_cb)
        self.__js = None
        self.__done = False
        self.__name = name

    def __js_cb(self, js):
        self.__js = js

    def __save_out(self):
        rs = RobotState()
        rs.joint_state = self.__js
        self.__save(self.__name, "", rs)

    def spin(self):
        while not self.__done and not rospy.is_shutdown():
            if self.__js is not None:
                self.__save_out()
                self.__done = True
            else:
                rospy.sleep(.1)

if "__main__" == __name__:
    rospy.init_node("grasp_saver")
    if len(argv) <= 1 or "" == argv[1]:
        rospy.logerr("You didn't enter a name.")
        exit(-1)
    gs = GraspSaver(argv[1])
    gs.spin()

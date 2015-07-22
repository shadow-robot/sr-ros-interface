#!/usr/bin/env python

import rospy
from shape_msgs.msg import SolidPrimative


class SrFastGrasp:
    def __init__(self):
        pass

    def start(self):
        pass

    def __bounding_box_cb(self, request):
        rospy.loginfo("got box")

if "__main__" == __name__:
    rospy.init_node('sr_fast_grasp')
    grasp = SrFastGrasp()
    grasp.start()
    rospy.spin()

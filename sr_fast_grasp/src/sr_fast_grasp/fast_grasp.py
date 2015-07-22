#!/usr/bin/env python

import rospy
from shape_msgs.msg         import SolidPrimitive
from visualization_msgs.msg import Marker
from sr_robot_msgs.srv      import GetFastGraspFromBoundingBox

class SrFastGrasp:
    def __init__(self):
        self.__marker_pub = rospy.Publisher("visualisation_narker",
                                            Marker, queue_size=1)

    def __bounding_box_cb(self, request):
        rospy.loginfo("got box")
        marker = self.__get_marker_from_box(request.bounding_box)
        self.__send_marker_to_rviz(marker)

    def __get_marker_from_box(self, box):
        marker = Marker()
        marker.pose = box.pose.pose()
        marker.header.frame_id = box.header.frame_id

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rospy.rostime.Duration()
        marker.type = Marker.CUBE
        marker.ns = "sr_fast_grasp_target"
        marker.id = 0
        marker.action = Marker.ADD

    def __send_marker_to_rviz(self, marker):
        pass

if "__main__" == __name__:
    rospy.init_node('sr_fast_grasp')
    grasp = SrFastGrasp()
    rospy.spin()

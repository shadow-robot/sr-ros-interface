#!/usr/bin/env python

import rospy
from shape_msgs.msg         import SolidPrimitive
from visualization_msgs.msg import Marker
from sr_robot_msgs.srv      import GetFastGraspFromBoundingBox
from moveit_msgs.msg        import Grasp
class SrFastGrasp:
    def __init__(self):
        self.__marker_pub   = rospy.Publisher("visualization_marker",
                                              Marker, queue_size=1)
        self.__grasp_server = rospy.Service("grasp_from_bounding_box",
                                            GetFastGraspFromBoundingBox,
                                            self.__bounding_box_cb)
        self.__default_grasp = 'super_amazing_grasp'

    def __bounding_box_cb(self, request):
        box = request.bounding_box
        pose = request.pose

        if SolidPrimitive.BOX != box.type:
            rospy.logerr("Bounding volume must be a BOX.")
            return None

        return self.__select_grasp(box,pose)

    def __select_grasp(self, box, pose):
        major_axis = self.__get_major_axis(box)
        rospy.loginfo(major_axis)
        return Grasp()

    def __get_major_axis(self, box):
        m = max(box.dimensions)
        max_index = [i for i,j in enumerate(box.dimensions) if j == m]
        return max_index[-1]  # Get the LAST axis with max val.

    def __send_marker_to_rvix(self, box, pose):
        marker = self.__get_marker_from_box(box,pose)
        self.__marker_pub.publish(marker)

    def __get_marker_from_box(self, box, pose):
        marker = Marker()
        marker.pose = pose.pose
        marker.header.frame_id = pose.header.frame_id

        marker.scale.x = box.dimensions[SolidPrimitive.BOX_X]
        marker.scale.y = box.dimensions[SolidPrimitive.BOX_Y]
        marker.scale.z = box.dimensions[SolidPrimitive.BOX_Z]

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime = rospy.rostime.Duration()
        marker.type = Marker.CUBE
        marker.ns = "sr_fast_grasp_target"
        marker.id = 0
        marker.action = Marker.ADD
        return marker


if "__main__" == __name__:
    rospy.init_node('sr_fast_grasp')
    grasp = SrFastGrasp()
    rospy.spin()

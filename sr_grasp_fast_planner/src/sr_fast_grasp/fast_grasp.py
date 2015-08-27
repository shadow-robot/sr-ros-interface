#!/usr/bin/env python

import numpy
from copy import deepcopy

import rospy
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
from sr_robot_msgs.srv import GetFastGraspFromBoundingBox
from moveit_msgs.msg import Grasp
from moveit_commander import MoveGroupCommander
from moveit_msgs.srv import GetRobotStateFromWarehouse as GetState
from moveit_msgs.srv import GetPositionIK


def quaternion_from_matrix(matrix, isprecise=False):
    M = numpy.array(matrix, dtype=numpy.float64, copy=False)[:4, :4]
    if isprecise:
        q = numpy.empty((4, ))
        t = numpy.trace(M)
        if t > M[3, 3]:
            q[0] = t
            q[3] = M[1, 0] - M[0, 1]
            q[2] = M[0, 2] - M[2, 0]
            q[1] = M[2, 1] - M[1, 2]
        else:
            i, j, k = 1, 2, 3
            if M[1, 1] > M[0, 0]:
                i, j, k = 2, 3, 1
            if M[2, 2] > M[i, i]:
                i, j, k = 3, 1, 2
            t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
            q[i] = t
            q[j] = M[i, j] + M[j, i]
            q[k] = M[k, i] + M[i, k]
            q[3] = M[k, j] - M[j, k]
        q *= 0.5 / math.sqrt(t * M[3, 3])
    else:
        m00 = M[0, 0]
        m01 = M[0, 1]
        m02 = M[0, 2]
        m10 = M[1, 0]
        m11 = M[1, 1]
        m12 = M[1, 2]
        m20 = M[2, 0]
        m21 = M[2, 1]
        m22 = M[2, 2]
        # symmetric matrix K
        K = numpy.array([[m00-m11-m22, 0.0,         0.0,         0.0],
                         [m01+m10,     m11-m00-m22, 0.0,         0.0],
                         [m02+m20,     m12+m21,     m22-m00-m11, 0.0],
                         [m21-m12,     m02-m20,     m10-m01,     m00+m11+m22]])
        K /= 3.0
        # quaternion is eigenvector of K that corresponds to largest eigenvalue
        w, V = numpy.linalg.eigh(K)
        q = V[[3, 0, 1, 2], numpy.argmax(w)]
    if q[0] < 0.0:
        numpy.negative(q, q)
    return q


class SrFastGrasp:
    def __init__(self):
        self.__marker_pub = rospy.Publisher("visualization_marker",
                                            Marker, queue_size=1)
        self.__grasp_server = rospy.Service("grasp_from_bounding_box",
                                            GetFastGraspFromBoundingBox,
                                            self.__bounding_box_cb)
        self.__default_grasp = 'super_amazing_grasp'
        self.__get_state = rospy.ServiceProxy(
            '/grasp_warehouse/get_robot_state', GetState)

        hand_group = rospy.get_param("~hand_group", "right_hand")
        arm_group = rospy.get_param("~arm_group", "right_arm")

        self.__group = MoveGroupCommander(hand_group)
        self.__arm_g = MoveGroupCommander(arm_group)
        self.__ik = rospy.ServiceProxy("compute_ik", GetPositionIK)

    def __modify_grasp_pose(self, grasp, pose):
        """
        Aligns grasp with axis from origin to center of object.
        A crude way to make a vaguely sane orientation for the hand
        that seems to more or less work.
        """

        v1 = numpy.array([pose.pose.position.x,
                          pose.pose.position.y,
                          pose.pose.position.z])
        v1_length = numpy.linalg.norm(v1)

        v1 = v1/v1_length

        v2 = [1, 0, -v1[0]/v1[2]]
        v2 = v2/numpy.linalg.norm(v2)

        v3 = numpy.cross(v1, v2)
        v3 = v3/numpy.linalg.norm(v3)

        m = [
            [v3[0], v1[0], v2[0]],
            [v3[1], v1[1], v2[1]],
            [v3[2], v1[2], v2[2]]
        ]

        q = quaternion_from_matrix(m)

        grasp.grasp_pose = deepcopy(pose)

        grasp.grasp_pose.pose.orientation.x = q[0]
        grasp.grasp_pose.pose.orientation.y = q[1]
        grasp.grasp_pose.pose.orientation.z = q[2]
        grasp.grasp_pose.pose.orientation.w = q[3]

    def __bounding_box_cb(self, request):
        box = request.bounding_box
        pose = request.pose
        if SolidPrimitive.BOX != box.type:
            rospy.logerr("Bounding volume must be a BOX.")
            return None
        self.__send_marker_to_rviz(box, pose)
        grasp_name = self.__select_grasp()
        grasp = self.__get_grasp(grasp_name)

        self.__modify_grasp_pose(grasp, pose)

        return grasp

    def __select_grasp(self):
        return self.__default_grasp

    def __get_grasp(self, name):
        try:
            open_state = self.__get_state(name + "_open", "").state
            closed_state = self.__get_state(name + "_closed", "").state
        except:
            rospy.logfatal("Couldn't get grasp pose from db.")
            return Grasp()

        try:
            self.__group.set_start_state_to_current_state()
            pre_pose = self.__group.plan(open_state.joint_state)
            self.__group.set_start_state(open_state)
            pose = self.__group.plan(closed_state.joint_state)
        except:
            rospy.logfatal("Couldn't plan grasp trajectories.")
            return Grasp()

        grasp = Grasp()
        grasp.id = name
        grasp.pre_grasp_posture = pre_pose.joint_trajectory
        grasp.grasp_posture = pose.joint_trajectory

        grasp.pre_grasp_approach.desired_distance = 0.2
        grasp.pre_grasp_approach.min_distance = 0.1
        grasp.pre_grasp_approach.direction.vector.x = 0
        grasp.pre_grasp_approach.direction.vector.y = -1
        grasp.pre_grasp_approach.direction.vector.z = 0

        return grasp

    def __get_major_axis(self, box):
        m = max(box.dimensions)
        max_index = [i for i, j in enumerate(box.dimensions) if j == m]
        return max_index[-1]  # Get the LAST axis with max val.

    def __send_marker_to_rviz(self, box, pose):
        marker = self.__get_marker_from_box(box, pose)
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
        marker.color.a = 0.5

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

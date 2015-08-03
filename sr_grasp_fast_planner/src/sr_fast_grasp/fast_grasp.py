#!/usr/bin/env python

import rospy
from shape_msgs.msg         import SolidPrimitive
from visualization_msgs.msg import Marker
from sr_robot_msgs.srv      import GetFastGraspFromBoundingBox
from moveit_msgs.msg        import Grasp
from moveit_commander       import MoveGroupCommander
from geometry_msgs.msg      import Pose
from moveit_msgs.srv import GetRobotStateFromWarehouse as GetState
from sensor_msgs.msg import JointState
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK
import numpy

def quaternion_from_matrix(matrix, isprecise=False):
    """Return quaternion from rotation matrix.

    If isprecise is True, the inumpyut matrix is assumed to be a precise rotation
    matrix and a faster algorithm is used.

    >>> q = quaternion_from_matrix(numpy.identity(4), True)
    >>> numpy.allclose(q, [1, 0, 0, 0])
    True
    >>> q = quaternion_from_matrix(numpy.diag([1, -1, -1, 1]))
    >>> numpy.allclose(q, [0, 1, 0, 0]) or numpy.allclose(q, [0, -1, 0, 0])
    True
    >>> R = rotation_matrix(0.123, (1, 2, 3))
    >>> q = quaternion_from_matrix(R, True)
    >>> numpy.allclose(q, [0.9981095, 0.0164262, 0.0328524, 0.0492786])
    True
    >>> R = [[-0.545, 0.797, 0.260, 0], [0.733, 0.603, -0.313, 0],
    ...      [-0.407, 0.021, -0.913, 0], [0, 0, 0, 1]]
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.19069, 0.43736, 0.87485, -0.083611])
    True
    >>> R = [[0.395, 0.362, 0.843, 0], [-0.626, 0.796, -0.056, 0],
    ...      [-0.677, -0.498, 0.529, 0], [0, 0, 0, 1]]
    >>> q = quaternion_from_matrix(R)
    >>> numpy.allclose(q, [0.82336615, -0.13610694, 0.46344705, -0.29792603])
    True
    >>> R = random_rotation_matrix()
    >>> q = quaternion_from_matrix(R)
    >>> is_same_transform(R, quaternion_matrix(q))
    True
    >>> R = euler_matrix(0.0, 0.0, numpy.pi/2.0)
    >>> numpy.allclose(quaternion_from_matrix(R, isprecise=False),
    ...                quaternion_from_matrix(R, isprecise=True))
    True

    """
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
        self.__marker_pub   = rospy.Publisher("visualization_marker",
                                              Marker, queue_size=1)
        self.__grasp_server = rospy.Service("grasp_from_bounding_box",
                                            GetFastGraspFromBoundingBox,
                                            self.__bounding_box_cb)
        self.__default_grasp = 'super_amazing_grasp'
        self.__get_state = rospy.ServiceProxy(
            '/moveit_warehouse_services/get_robot_state', GetState)
        self.__group = MoveGroupCommander("right_hand")
        self.__arm_g = MoveGroupCommander("right_arm")
        self.__ik = rospy.ServiceProxy("compute_ik", GetPositionIK)


    def __bounding_box_cb(self, request):
        box  = request.bounding_box
        pose = request.pose
        if SolidPrimitive.BOX != box.type:
            rospy.logerr("Bounding volume must be a BOX.")
            return None
        self.__send_marker_to_rviz(box, pose)
        grasp_name = self.__select_grasp()
        #grasp = self.__get_grasp(grasp_name)
        #grasp.grasp_pose = self.__orient_grasp(box, pose)


        self.__arm_g.set_start_state_to_current_state()

        v1 = numpy.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
        v1_length = numpy.linalg.norm(v1)

        v1 = v1/v1_length

        v2 = [1,0,-v1[0]/v1[2]]
        v2 = v2/numpy.linalg.norm(v2)

        v3 = numpy.cross(v1,v2)
        v3 = v3/numpy.linalg.norm(v3)

        m = [
            [v1[0],v2[0],v3[0]],
            [v1[1],v2[1],v3[1]],
            [v1[2],v2[2],v3[2]]
        ]



        q= quaternion_from_matrix(m)

        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]


        self.__arm_g.plan(pose)

        return Grasp()

    def __select_grasp(self):
        return self.__default_grasp

    def __get_grasp(self, name):
        open_state   = self.__get_state(name + "_open", "").state
        closed_state  = self.__get_state(name + "_closed", "").state

        self.__group.set_start_state_to_current_state()
        pre_pose = self.__group.plan(open_state.joint_state)

        self.__group.set_start_state(open_state)
        pose = self.__group.plan(closed_state.joint_state)

        grasp = Grasp()
        grasp.id = name
        #grasp.pre_grasp_posture = pre_pose.joint_trajectory
        #grasp.grasp_posture     = pose.joint_trajectory
        #fill grasp

        return grasp

    def __orient_grasp(self, box, pose):
        major_axis = self.__get_major_axis(box)
        if   2 == major_axis:  # z
            pass
        elif 1 == major_axis:  # y
            pass
        else:  # x
            pass

        return pose


    def __get_major_axis(self, box):
        m = max(box.dimensions)
        max_index = [i for i,j in enumerate(box.dimensions) if j == m]
        return max_index[-1]  # Get the LAST axis with max val.

    def __send_marker_to_rviz(self, box, pose):
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

#!/usr/bin/env python
from math import log10, floor
from unittest import TestCase

import rospy
from sr_robot_msgs.srv import GetFastGraspFromBoundingBox \
    as GetGrasp
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped


def round_sig(x, sig):
    return round(x, sig-int(floor(log10(x)))-1)


class TestFastGraspPlanner(TestCase):
    def setUp(self):
        self._planning_service = rospy.ServiceProxy(
            '/grasp_from_bounding_box', GetGrasp)

    # For some reason, calling wait for service crashes rostest :/
    # def test_service_running(self):
    #     self.assertFalse(self._planning_service is None)

    #     try:
    #         rospy.wait_for_service('/grasp_from_bounding_box', 1)
    #     except rospy.ServiceException as e:
    #         self.assertTrue(False)
    #         pass

    def make_box_and_pose(self):
        pose = PoseStamped()
        pose.pose.position.x = 0.5
        pose.pose.position.y = 0.5
        pose.pose.position.z = 0.7

        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 0
        pose.header.frame_id = 'world'

        box = SolidPrimitive()
        box.type = 1
        box.dimensions = [0.05, 0.05, 0.05]
        return box, pose

    def test_grasp(self):
        box, pose = self.make_box_and_pose()
        grasp = self._planning_service(box, pose).grasp

        self.assertEqual(grasp.id, "super_amazing_grasp")

        self.assertEqual(grasp.grasp_pose.pose.position.x, 0.5)
        self.assertEqual(grasp.grasp_pose.pose.position.y, 0.5)
        self.assertEqual(grasp.grasp_pose.pose.position.z, 0.7)

        self.assertEqual(round_sig(grasp.grasp_pose.pose.orientation.x, 5),
                         round_sig(0.396609862374, 5))
        self.assertEqual(round_sig(grasp.grasp_pose.pose.orientation.y, 5),
                         round_sig(0.443462541797, 5))
        self.assertEqual(round_sig(grasp.grasp_pose.pose.orientation.z, 5),
                         round_sig(0.770688050305, 5))
        self.assertEqual(round_sig(grasp.grasp_pose.pose.orientation.w, 5),
                         round_sig(0.2282137599, 5))

        self.assertEqual(1, 1)


if __name__ == '__main__':
    import rostest

    rospy.sleep(20)

    # rostest.rosrun("test_service_running",
    #                "test_fast_grasp_planner",
    #                TestFastGraspPlanner)

    rostest.rosrun("test_grasp_selection",
                   'test_fast_grasp_planner',
                   TestFastGraspPlanner)

#!/usr/bin/env python
import rospy
from sr_robot_msgs.srv import GetFastGraspFromBoundingBox \
    as GetGrasp
from unittest import TestCase

class TestFastGraspPlanner(TestCase):
    def setUp(self):
        self._planning_service = rospy.ServiceProxy(
            '/grasp_from_bounding_box', GetGrasp)

        pass

    def test_service_running(self):
        try:
            self._planning_service.wait_for_service(30)
        except rospy.ServiceException as e:
            pass

        self.assertFalse(self._planning_service is None)

    def test_grasp_selection(self):
        # make simple box
        # test grasp name
        # test trajectories? somehow?
        # test pose


        self.assertEqual(1,1)


if __name__ == '__main__':
    import rostest

    rostest.rosrun("test_service_running",
                   "test_fast_grasp_planner",
                   TestFastGraspPlanner)

    rostest.rosrun("test_grasp_selection",
                   'test_fast_grasp_planner',
                   TestFastGraspPlanner)

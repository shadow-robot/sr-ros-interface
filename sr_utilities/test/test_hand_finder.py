#!/usr/bin/env python

import rospy
import rostest
import unittest
from sr_utilities.hand_finder import HandFinder


class TestHandFinder(unittest.TestCase):
    def test_one_hand_finder(self):
        rospy.set_param("hand/joint_prefix/1", "rh_")
        rospy.set_param("hand/mapping/1", "rh")

        hand_finder = HandFinder()
        self.assertIsNotNone(hand_finder.get_hand_parameters(),
                             "Parameters extracted.")
        self.assertIsNotNone(hand_finder.get_hand_joints(),
                             "Joints extracted.")
        self.assertIsNotNone(hand_finder.get_calibration_path(),
                             "Calibration extracted.")
        self.assertIsNotNone(hand_finder.get_hand_control_tuning(),
                             "Control tuning parameters extracted.")


if __name__ == "__main__":
    rospy.init_node("test_hand_finder")
    rostest.rosrun("sr_utilities", "test_hand_finder", TestHandFinder)

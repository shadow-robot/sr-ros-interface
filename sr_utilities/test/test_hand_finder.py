#!/usr/bin/env python

import rospy
import rostest
import unittest
from sr_utilities.hand_finder import HandFinder


class TestHandFinder(unittest.TestCase):
    def test_no_hand_finder(self):
        if rospy.has_param("hand"):
            rospy.delete_param("hand")

        hand_finder = HandFinder()

        self.assertEqual(len(hand_finder.get_hand_parameters().joint_prefix),
                         0, "correct parameters without a hand")
        self.assertEqual(len(hand_finder.get_hand_parameters().mapping),
                         0, "correct parameters without a hand")
        self.assertEqual(len(hand_finder.get_hand_joints()), 0,
                         "correct joints without a hand")
        self.assertEqual(len(hand_finder.get_calibration_path()), 0,
                         "correct calibration path without a hand")
        self.assertEqual(len(hand_finder.get_hand_control_tuning().
                             friction_compensation), 0,
                         "correct tuning without a hands")
        self.assertEqual(len(hand_finder.get_hand_control_tuning().
                             host_control), 0,
                         "correct tuning without a hands")
        self.assertEqual(len(hand_finder.get_hand_control_tuning().
                             motor_control), 0,
                         "correct tuning without a hands")

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

    def test_two_hand_finder(self):
        rospy.set_param("hand/joint_prefix/1", "rh_")
        rospy.set_param("hand/mapping/1", "rh")
        rospy.set_param("hand/joint_prefix/2", "lh_")
        rospy.set_param("hand/mapping/2", "lh")

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

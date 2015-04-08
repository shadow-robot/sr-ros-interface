#!/usr/bin/python

# Copyright 2014 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy
import threading

from sr_robot_msgs.msg import sendupdate, joint
from sr_hand.shadowhand_ros import ShadowHand_ROS
from sr_hand.grasps_interpoler import GraspInterpoler
from sr_hand.Grasp import Grasp
from sr_robot_commander.sr_robot_commander import SrRobotCommander

"""
Module to provide a quick and easy way to script the Shadow Hand.
This is done via creating a short script with a Commander object then calling
methods on the commander to script the motion.

Basic script will look like:

    #!/usr/bin/python
    import rospy
    from sr_hand.shadowhand_commander import Commander

    rospy.init_node('basic_example')
    c = Commander()

    c.move_hand({
        "THJ1": 0, "THJ2": 6, "THJ3": 10, "THJ4": 37, "THJ5": 9,
        "FFJ0": 21, "FFJ3": 26, "FFJ4": 0,
        "MFJ0": 18, "MFJ3": 24, "MFJ4": 0,
        "RFJ0": 30, "RFJ3": 16, "RFJ4": 0,
        "LFJ0": 30, "LFJ3": 0, "LFJ4": -10, "LFJ5": 10
    })
    rospy.sleep(3.0)
    

See the examples directory in the package sr_examples.
"""

class Commander(SrRobotCommander):
    def __init__(self):

        # TODO Add code which reads correct MoveIt group name for hand
        # Calls constructor of the parent class
        super(Commander, self).__init__("")

        # Shadow hand setup
        self.hand = ShadowHand_ROS()

    def move_hand(self, command):
        """
        Move the Shadow Hand.

        This call is non blocking. It will return before the motion has finished.

        @param command - Dictionary of joint names in the keys and angles in
        degrees in the values.
        """
        self._set_joint_value_target(command, wait_result=False)

    def get_hand_position(self):
        """
        Returns a dictionary with the position of each joint in degrees.
        """
        return dict(self.hand.read_all_current_positions())

    def get_hand_velocity(self):
        """
        Returns a dictionary with the velocity of each joint in degrees/s.
        """
        return dict(self.hand.read_all_current_velocities())

    def get_hand_effort(self):
        """
        Returns a dictionary with the effort of each joint. Currently in ADC units, as no calibration is performed on
        the strain gauges.
        """
        return dict(self.hand.read_all_current_efforts())

    def get_tactile_type(self):
        """
        Returns a string indicating the type of tactile sensors present. Possible values are: PST, biotac, UBI0 .
        """
        return self.hand.get_tactile_type()

    def get_tactile_state(self):
        """
        Returns an object containing tactile data. The structure of the data is different for every tactile_type .
        """
        return self.hand.get_tactile_state()
    


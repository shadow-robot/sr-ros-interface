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

from sr_robot_msgs.msg import BiotacAll, ShadowPST, UBI0All


class TactileReceiver(object):
    """
    Module that receives the tactile values.
    """

    def __init__(self, prefix=""):
        """
        Receives the tactile information from a Shadow Hand
        @param prefix - prefix for the tactile topic
        """
        # appends trailing slash if necessary
        if not prefix.endswith("/"):
            prefix += "/"

        self._prefix = prefix

        self.tactile_type = self.find_tactile_type()
        self.tactile_state = None

        if self.tactile_type == "PST":
            self.tactile_listener = rospy.Subscriber(prefix+"tactile", ShadowPST, self.tactile_callback, queue_size=1)
        elif self.tactile_type == "biotac":
            self.tactile_listener = rospy.Subscriber(prefix+"tactile", BiotacAll, self.tactile_callback, queue_size=1)
        elif self.tactile_type == "UBI0":
            self.tactile_listener = rospy.Subscriber(prefix+"tactile", UBI0All, self.tactile_callback, queue_size=1)

    def find_tactile_type(self):
        try:
            rospy.wait_for_message(self._prefix+"tactile", ShadowPST, timeout=0.2)
            return "PST"
        except (rospy.ROSException, rospy.ROSInterruptException):
            pass

        try:
            rospy.wait_for_message(self._prefix+"tactile", BiotacAll, timeout=0.2)
            return "biotac"
        except (rospy.ROSException, rospy.ROSInterruptException):
            pass

        try:
            rospy.wait_for_message(self._prefix+"tactile", UBI0All, timeout=0.2)
            return "UBI0"
        except (rospy.ROSException, rospy.ROSInterruptException):
            rospy.logwarn("No tactile topic found. This is normal for a simulated hand")

        return None

    def tactile_callback(self, tactile_msg):
        self.tactile_state = tactile_msg

    def get_tactile_type(self):
        return self.tactile_type

    def get_tactile_state(self):
        return self.tactile_state

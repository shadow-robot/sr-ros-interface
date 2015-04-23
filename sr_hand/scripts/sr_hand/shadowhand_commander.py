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

class Commander(object):
    def __init__(self):
        rospy.logwarn("The class Commander in package sr_hand is deprecated. "
                      "Please use SrHandCommander from package sr_robot_commander")

        # Mutex to control thread access to certain operations
        self.mutex = threading.Lock()

        # This is used to store the grasp interpolators for the different threads.
        self.grasp_interpolators = {}

        # Shadow hand setup
        self.hand = ShadowHand_ROS()

        # Period in seconds between interpolated commands (e.g. if we want an interpolation time of 1s and have a period of 0.1,
        # the interpoler will use 10 steps
        self.hand_interpolation_period = 0.01
        if self.hand.check_hand_type() == "gazebo":
            # Interpolation is slower under simulation compared to real hand
            self.hand_interpolation_period = 0.1


    def move_hand(self, command):
        """
        Move the Shadow Hand.

        This call is non blocking. It will return before the motion has finished.

        If called with interpolation time of 0.0 the joints will move to the target position
        at maximum velocity (i.e. the max. vel. allowed by the position controller for that joint)

        If called with interpolation time greater than self.hand_interpolation_period,
        a separate thread will deal with the interpolation time and will keep
        sending targets to the hand until the last target is sent.

        If move_hand() is called again before the previous movements have finished,
        the new call will override only the conciding joints. The others will keep moving
        to their previous targets at their previous velocity.

        @param command - Dictionary of joint names in the keys and angles in
        degrees in the values. The key interpolation_time gives the time in seconds that
        the movement will last.
        """

        # Copy the dictionary, so that we will not affect the original user command
        joints = dict(command)

        interpolation_time = 0.0
        if 'interpolation_time' in joints:
            interpolation_time = joints['interpolation_time']
            del joints['interpolation_time']

        if interpolation_time == 0.0:
            self.mutex.acquire()
            self._prune_interpolators(joints)
            self.mutex.release()
            self.hand.sendupdate_from_dict(joints)
        else:
            threading.Thread(target=self._move_hand_interpolation,
                             args=(joints, interpolation_time)).start()


    def _move_hand_interpolation(self, joints, interpolation_time=1.0):
        """
        Remove the coinciding joints from the currently running interpolators.
        It actually removes the joints from the 'grasp_to' list of the interpolator.

        @param joints - Dictionary of joint names in the keys and angles in
        degrees in the values.
        @param interpolation_time - A float. Interpolation time in seconds.
        """

        rospy.logdebug("call interpolation")
        thread_name = threading.current_thread().name
        try:
            self.mutex.acquire()
            self._prune_interpolators(joints)
            self.mutex.release()

            rospy.logdebug("start interpolation")
            current_grasp = Grasp()
            current_grasp.joints_and_positions = self.hand.read_all_current_positions()
            target_grasp = Grasp()
            target_grasp.joints_and_positions = joints

            interpolator = GraspInterpoler(current_grasp, target_grasp)
            self.grasp_interpolators[thread_name] = interpolator

            r = rospy.Rate(1.0 / self.hand_interpolation_period)

            for interpolation in range (1, int(interpolation_time / self.hand_interpolation_period) + 1):
                self.mutex.acquire()
                targets_to_send = interpolator.interpolate(100.0 * interpolation * self.hand_interpolation_period / interpolation_time)
                self.mutex.release()
                self.hand.sendupdate_from_dict(targets_to_send)
                #rospy.loginfo("sent cmd n: %s" %(str(interpolation), ))
                r.sleep()
        finally:
            self.mutex.acquire()
            self.grasp_interpolators.pop(thread_name, None)
            self.mutex.release()
            rospy.logdebug("end interpolation")


    def _prune_interpolators(self, joints):
        """
        Remove the coinciding joints from the currently running interpolators.
        It actually removes the joints from the 'grasp_to' list of the interpolator.

        @param joints - Dictionary of joint names in the keys and angles in
        degrees in the values.
        """
        rospy.logdebug("Call prune from thread %s", threading.current_thread().name )
        for thread_id in self.grasp_interpolators.keys():
            for joint_name in joints.keys():
                self.grasp_interpolators[thread_id].grasp_to.joints_and_positions.pop(joint_name, None)
                rospy.logdebug("Prune joint %s thread %s", joint_name, thread_id )


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
        Returns a dictionary with the effort of each joint. Currently in ADC units, as no calibration is performed on the strain gauges.
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



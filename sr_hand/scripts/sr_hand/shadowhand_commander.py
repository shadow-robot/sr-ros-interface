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

import roslib;
import rospy
import threading

from sr_robot_msgs.msg import sendupdate, joint
from sr_hand.shadowhand_ros import ShadowHand_ROS
from Grasp import Grasp

"""
Module to provide a quick and easy way to script the Shadow Hand.
This is done via creating a short script with a Commander object then calling
methods on the commander to script the motion.

Basic script will look like:

    #!/usr/bin/python
    import roslib;
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

Take note that the blocking behaviour of move_pos and move_axis (return when
move is complete) is different to move_hand (returns before motion completes).
Therefore when scripting motions for the arm and hand together you are advised
to move the hand, then the arm and then add a sleep if needed to syncronise.
"""

class Commander(object):
    def __init__(self):
        # Mutex to avoid two calls of self.move_hand to be active at the same time
        self.mutex = threading.Lock()
        # Mutex to control access to the variable self.hand_mvt_stopped
        self.mutex_stop_mov = threading.Lock()
        # This is used to stop the currently running (in case there is one) hand interpolation thread. We override the still running command with the new one.
        self.hand_mvt_stopped = True

        # Shadow hand setup
        self.hand = ShadowHand_ROS()

        # Period in seconds between interpolated commands (e.g. if we want an interpolation time of 1s and have a period of 0.1, 
        # the interpoler will use 10 steps
        self.hand_interpolation_period = 0.01
        if self.hand.check_hand_type() == "gazebo":
            # Interpolation is far too slow under simulation compared to real hand
            self.hand_interpolation_period = 0.1


    def move_hand(self, joints):
        """
        Move the Shadow Hand.

        This is call is non blocking. It will return before the motion has finished.

        If called with interpolation time of 0.0 or smaller than
        self.hand_interpolation_period, sending another target while the hand
        is still moving will cause the new move to take over). So you will need
        to add sleeps to control the motion.

        If called with interpolation time greater than self.hand_interpolation_period,
        a separate thread will deal with the interpolation time and will keep 
        sending targets to the hand until the last target is sent.
        Sending another command while the previous one is still going will cause the new
        move_hand call to block until the previous 
        move to take over). So you will need to add sleeps to control the
        motion.

        @param joints - Dictionary of joint names in the keys and angles in
        degrees in the values.
        """
        #This allows to end the self.hand_interpolation_thread in case it is running
        self.mutex_stop_mov.acquire()
        self.hand_mvt_stopped = True
        self.mutex_stop_mov.release()
        
        interpolation_time = 0.0
        if 'interpolation_time' in joints:
            interpolation_time = joints['interpolation_time']

        if interpolation_time == 0.0:
            self.mutex.acquire()
            self.hand.sendupdate_from_dict(joints)
            self.mutex.release()
        else:
            self.hand_interpolation_thread = threading.Thread(
                    target=self._move_hand_interpolation,
                    args=(joints, interpolation_time)).start()

    def _move_hand_interpolation(self, joints, interpolation_time=1.0):
        rospy.logdebug("call interpol")
        self.mutex.acquire()
        self.hand_mvt_stopped = False
        try:
            rospy.logdebug("start interpol")
            current_grasp = Grasp()
            current_grasp.joints_and_positions = self.hand.read_all_current_positions()
            target_grasp = Grasp()
            target_grasp.joints_and_positions = joints

            interpoler = self.hand.create_grasp_interpoler(current_grasp, target_grasp)
            r = rospy.Rate(1.0 / self.hand_interpolation_period)
            for interpolation in range (1, int(interpolation_time / self.hand_interpolation_period) + 1):
                #This allows to end this thread in case a new move_hand command is called
                self.mutex_stop_mov.acquire()
                if self.hand_mvt_stopped:
                    self.mutex_stop_mov.release()
                    rospy.loginfo("Interpolation overriden. A new hand command has overriden the ongoing movement")
                    return
                self.mutex_stop_mov.release()

                targets_to_send = self.hand.grasp_interpoler.interpolate(100.0 * interpolation * self.hand_interpolation_period / interpolation_time)
                self.hand.sendupdate_from_dict(targets_to_send)
                #rospy.loginfo("sent cmd n: %s" %(str(interpolation), ))
                r.sleep()
        finally:
            self.mutex.release()
            rospy.logdebug("end interpol")

    def get_hand_position(self):
        return self.hand.get_joint_positions()

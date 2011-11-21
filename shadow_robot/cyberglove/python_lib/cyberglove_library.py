#!/usr/bin/env python
#
# Copyright 2011 Shadow Robot Company Ltd.
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
#
import roslib; roslib.load_manifest('cyberglove')

import time
import os
import math
import rospy
import subprocess
import threading
import rosgraph.masterapi
from sr_robot_msgs.msg import sendupdate, joint, joints_data
from sensor_msgs.msg import *

class Joint():
    def __init__(self, name="", motor="", min=0, max=90):
        self.name = name
        self.motor = motor
        self.min = min
        self.max = max

class Cyberglove:
    """
    Interface to the Cyberglove publisher.
    """
    def __init__(self, max_values = 2):
        self.joints = { "G_ThumbRotate": Joint(),
                        "G_ThumbMPJ": Joint(),
                        "G_ThumbIJ": Joint(),
                        "G_ThumbAb": Joint(),
                        "G_IndexMPJ": Joint(),
                        "G_IndexPIJ": Joint(),
                        "G_IndexDIJ": Joint(),
                        "G_MiddleMPJ": Joint(),
                        "G_MiddlePIJ": Joint(),
                        "G_MiddleDIJ": Joint(),
                        "G_MiddleIndexAb": Joint(),
                        "G_RingMPJ": Joint(),
                        "G_RingPIJ": Joint(),
                        "G_RingDIJ": Joint(),
                        "G_RingMiddleAb": Joint(),
                        "G_PinkieMPJ": Joint(),
                        "G_PinkiePIJ": Joint(),
                        "G_PinkieDIJ": Joint(),
                        "G_PinkieRingAb": Joint(),
                        "G_PalmArch": Joint(),
                        "G_WristPitch": Joint(),
                        "G_WristYaw": Joint() }

        self.raw_messages = []
        self.calibrated_messages = []
        self.max_values = max_values
        self.map = {}
        self.hasglove = 0
        self.isFirstMessage = True
        self.liste = 0
        self.raw = rospy.Subscriber('/cyberglove/raw/joint_states',JointState,self.callback_raw)
        self.calibrated = rospy.Subscriber('/cyberglove/calibrated/joint_states',JointState,self.callback_calibrated)
        threading.Thread(None, rospy.spin)
        if self.has_glove():
            time.sleep(1.0)
            self.createMap()
        else:
            raise Exception("No glove found")

    def callback_raw(self, data):
        """
        Adds the last values received to the list of raw values
        @param data: the message which called the callback
        """
        self.addValue(self.raw_messages, data)

    def callback_calibrated(self, data):
        """
        Adds the last values received to the list of calibrated values
        @param data: the message which called the callback
        """
        self.addValue(self.calibrated_messages, data)

    def addValue(self, vector, value):
        """
        Fills a vector with the received values, and replaces the old values
        when the vector is full
        @param vector : the vector to fill (raw or calibrated)
        @param value : the value to add
        """
        if len(vector) < self.max_values:
            vector.append(value)
        else:
            vector.pop(0)
            vector.append(value)

    def createMap(self):
        """
        Maps the name of the joints to their index in the message
        """
        for index in range(0, len(self.raw_messages[0].name)):
            self.map[self.raw_messages[0].name[index]] = index

    def read_raw_average_value(self, joint_name):
        """
        return the raw value of a given joint

        @param joint_name: the name of the glove of the Cyberglove
        """
        raw_value =  0
        joint_index = self.map[joint_name]
        for index in range(0, len(self.raw_messages)):
            raw_value = raw_value + self.raw_messages[index].position[joint_index]

        raw_value = raw_value / len(self.raw_messages)

        return raw_value

    def read_calibrated_average_value(self, joint_name):
        """
        return the current positions for the given joint_name

        @param joint_name:  the name of the joint
        @return: the corresponding position
        """

        calibrated_value = 0

        joint_index = self.map[joint_name]
        for index in range(0, len(self.calibrated_messages)):
            calibrated_value = calibrated_value + self.calibrated_messages[index].position[joint_index]

        calibrated_value = calibrated_value / len(self.calibrated_messages)

        return calibrated_value

    def get_joints_names(self):
        """
        Return an array containing the Cyberglove joints names

        @return: the joints names array
        """
        return self.joints.keys()

    def has_glove(self):
        """
        @return: True if a cyberglove is detected by ROS
        """
        if not self.hasglove == 0:
            return self.hasglove
        self.hasglove = False
        if self.liste == 0:
            master = rosgraph.masterapi.Master('/rostopic')
            self.liste = master.getPublishedTopics('/')
        for topic_typ in self.liste :
            for topic in topic_typ:
                if '/cyberglove' in topic :
                    self.hasglove = True
        return self.hasglove

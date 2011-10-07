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

import roslib; roslib.load_manifest('sr_friction_compensation')
import rospy

from etherCAT_hand_lib import EtherCAT_Hand_Lib
#from sensor_msgs import JointState
from std_msgs.msg import Float64

import time

epsilon = 0.02

class SrFrictionCompensation(object):
    """
    """

    def __init__(self):
        """
        """
        self.lib = EtherCAT_Hand_Lib()

        self.joint_name = "FFJ3"
        if rospy.has_param("joint_name"):
            self.joint_name = rospy.get_param("joint_name")

        self.min = 0.0
        if rospy.has_param("min"):
            self.min = rospy.get_param("min")

        self.max = 1.57
        if rospy.has_param("max"):
            self.min = rospy.get_param("max")

        self.sign = -1
        sign_param_name = rospy.search_param('sign')
        self.sign = int(rospy.get_param(sign_param_name))
        rospy.loginfo("sign: "+str(self.sign))

        self.publisher = rospy.Publisher("/sr_friction_compensation/"+self.joint_name, Float64)
        self.rate = rospy.Rate(100)

        self.forces = []
        self.positions = []

    def run(self):
        self.lib.activate()
        time.sleep(0.5)

        self.move_to_start()

        time.sleep(1)

        self.record_map( -self.sign )

    def move_to_start(self):
        msg = Float64()

        #200 should be big enough to move the finger to the end of its range
        msg.data = self.sign * 250

        #try to get past the minimum
        rospy.loginfo("Moving to the starting position")
        while (self.lib.get_position(self.joint_name) > self.min) and not rospy.is_shutdown():
            self.publisher.publish(msg)
            self.rate.sleep()

        msg.data = 0.0
        for i in range(0,20):
            self.publisher.publish(msg)
            self.rate.sleep()

        rospy.loginfo("Starting position reached")

    def record_map(self, sign):
        rospy.loginfo("Recording map")
        while (self.lib.get_position(self.joint_name) < self.max) and not rospy.is_shutdown():
            force, pos = self.find_smallest_force(self.lib.get_position(self.joint_name), sign)
            if force != False:
                self.forces.append(force)
                self.positions.append(pos)

    def find_smallest_force(self, first_position, sign):
        msg = Float64()

        for force in range(0, 400):
            if rospy.is_shutdown():
                break

            if abs(self.lib.get_position(self.joint_name) - first_position) > epsilon:
                #ok, the finger moved, return the necessary force
                print "finger moved from ",first_position, " with force ", force
                return force, first_position
            else:
                msg.data = sign*force
                self.publisher.publish(msg)
            self.rate.sleep()

        return False, False


if __name__ == '__main__':
    rospy.init_node('sr_friction_compensation', anonymous=True)

    sr_fric = SrFrictionCompensation()
    sr_fric.run()

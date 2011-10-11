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
from pylab import plot, show

from scipy.optimize import leastsq
#from numpy import interp
import numpy
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

        self.max = 1.55
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
        self.interpolate_map()

        self.record_map( self.sign, False )
        self.interpolate_map()

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

    def record_map(self, sign, increasing = True):
        rospy.loginfo("Recording map")
        msg = Float64()

        if( increasing ):
            while (self.lib.get_position(self.joint_name) < self.max) and not rospy.is_shutdown():
                self.map_step(sign, msg)
        else:
            while (self.lib.get_position(self.joint_name) > self.min) and not rospy.is_shutdown():
                self.map_step(sign, msg)

    def map_step(self, sign, msg):
        force, pos = self.find_smallest_force(self.lib.get_position(self.joint_name), sign)
        if force != False:
            self.forces.append(force)
            self.positions.append(pos)
            msg.data = 0.0
            self.publisher.publish(msg)
            rospy.Rate(2).sleep()

    def find_smallest_force(self, first_position, sign):
        msg = Float64()

        for force in range(0, 400):
            if rospy.is_shutdown():
                break

            if abs(self.lib.get_position(self.joint_name) - first_position) > epsilon:
                #ok, the finger moved, return the necessary force
                measured_force = self.lib.get_effort(self.joint_name)
                print "finger moved from ",first_position, " with force ", measured_force
                return measured_force, first_position
            else:
                msg.data = sign*force
                self.publisher.publish(msg)
            self.rate.sleep()

        return False, False

    def interpolate_map(self):

        interp_pos = numpy.linspace(self.min, self.max, 10)
        fp = lambda v, x: v[0]+ v[1]*x

        v = self.linear_interp(-1, -1)
        print v

        plot(interp_pos, fp(v, interp_pos), '-')
        plot(self.positions, self.forces, 'o')

        show()

    def linear_interp(self, min_index, max_index):

        fp = lambda v, x: v[0]+ v[1]*x
        e = lambda v, x, y: (fp(v,x)-y)

        v0 = [3., 1, 4.]

        v = 0
        if min_index == -1 and max_index == -1:
            v, success = leastsq(e, v0, args=(self.positions,self.forces), maxfev=10000)
        else:
            v, success = leastsq(e, v0, args=(self.positions[min_index:max_index],self.forces[min_index:max_index]), maxfev=10000)
        return v


if __name__ == '__main__':
    rospy.init_node('sr_friction_compensation', anonymous=True)

    sr_fric = SrFrictionCompensation()
    sr_fric.run()

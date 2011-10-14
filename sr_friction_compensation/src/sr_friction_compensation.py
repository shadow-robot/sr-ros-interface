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
from pylab import plot, savefig

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
        if rospy.has_param("~joint_name"):
            self.joint_name = rospy.get_param("~joint_name")

        self.min = 0.0
        if rospy.has_param("~min"):
            self.min = rospy.get_param("~min")

        self.max = 1.55
        if rospy.has_param("~max"):
            self.max = rospy.get_param("~max")

        self.nb_repetition = 2
        if rospy.has_param("~nb_repetition"):
            self.nb_repetition = rospy.get_param("~nb_repetition")

        print " nb repetition: ", self.nb_repetition

        self.sign = -1
        if rospy.has_param("~sign"):
            self.sign = rospy.get_param("~sign")
        rospy.loginfo("sign: "+str(self.sign))

        self.publisher = rospy.Publisher("/sr_friction_compensation/"+self.joint_name, Float64)
        self.rate = rospy.Rate(100)

        self.forces = []
        self.positions = []

    def run(self):
        self.lib.activate()
        time.sleep(0.5)

        for i in range(0,self.nb_repetition):
            self.move_to_start()
            time.sleep(1)
            self.record_map( -self.sign )
            time.sleep(1)

            print "forces length: ", len(self.forces), " pos length: ", len(self.positions)

        self.interpolate_map(1)

        self.forces = []
        self.positions = []

        for i in range(0, self.nb_repetition):
            self.move_to_start(False)
            time.sleep(1)
            self.record_map( self.sign, False )
            time.sleep(1)
            print "forces length: ", len(self.forces), " pos length: ", len(self.positions)


        self.interpolate_map(2)

    def move_to_start(self, start_at_min = True):
        msg = Float64()

        #try to get past the minimum
        if start_at_min:
            rospy.loginfo("Moving to the starting position ("+str(self.min)+")")
        else:
            rospy.loginfo("Moving to the starting position ("+str(self.max)+")")
        if( start_at_min ):
            #250 should be big enough to move the finger to the end of its range
            msg.data = self.sign * 250
            while (self.lib.get_position(self.joint_name) > self.min) and not rospy.is_shutdown():
                self.publisher.publish(msg)
                self.rate.sleep()
        else:
            #250 should be big enough to move the finger to the end of its range
            msg.data = -self.sign * 250

            while (self.lib.get_position(self.joint_name) < self.max) and not rospy.is_shutdown():
                self.publisher.publish(msg)
                self.rate.sleep()

        msg.data = 0.0
        for i in range(0,20):
            self.publisher.publish(msg)
            self.rate.sleep()

        rospy.loginfo("Starting position reached")

    def record_map(self, sign, increasing = True):
        if increasing:
            rospy.loginfo("Recording increasing map, until "+str(self.max))
        else:
            rospy.loginfo("Recording decreasing map, until"+str(self.min))
        msg = Float64()

        if( increasing ):
            while (self.lib.get_position(self.joint_name) < self.max) and not rospy.is_shutdown():
                self.map_step(sign, msg)
        else:
            while (self.lib.get_position(self.joint_name) > self.min) and not rospy.is_shutdown():
                self.map_step(sign, msg, False)

    def map_step(self, sign, msg, increasing=True):
        #keep the tendon under tension
        min_force = 0.
        if increasing:
            min_force = 0.
        else:
            min_force = 70.
        force, pos = self.find_smallest_force(self.lib.get_position(self.joint_name), sign, min_force)
        if force != False:
            self.forces.append(force)
            self.positions.append(pos)
            msg.data = sign*min_force
            self.publisher.publish(msg)
            rospy.Rate(2).sleep()

    def find_smallest_force(self, first_position, sign, min_force):
        msg = Float64()

        for force in range(min_force, 400):
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

    def interpolate_map(self, index):

        interp_pos = numpy.linspace(self.min, self.max, 10)
        fp = lambda v, x: v[0]+ v[1]*x

        v = self.linear_interp(-1, -1)
        print v

        plot(interp_pos, fp(v, interp_pos), '-')

        print "forces length: ", len(self.forces), " pos length: ", len(self.positions)
        plot(self.positions, self.forces, 'o')

        savefig("friction_compensation_"+self.joint_name+".png")

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

#!/usr/bin/env python
# -*- coding: utf-8 -*-
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
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#By: Emilie JEAN-BAPTISTE
##Date:24 Juin 2011

import roslib; roslib.load_manifest('sr_automatic_pid_tuning')
import rospy

from sr_automatic_pid_tuning.communication_with_robot.robot_lib import Robot_Lib

class Partial_Movements(object):

    def __init__(self,joint_name):

        #Parameters
        self.rate=19.9
        self.SLEEP_TIME=0.5
        self.FREQ=0.3
        self.Maxval=80
        self.Minval=5
        self.Midval=40
        self.rate = rospy.Rate(self.rate)
        ##
        self.joint_name=joint_name
        self.movement_name="name_of_movement"
        ##
        self.robot_library= Robot_Lib()
        self.robot_library.init_publisher()
        return

    def publish_the_movement(self):
        """
        Send the movement on the publisher
        @return: nothing
        """
        print(self.movement_name)

        for itr in range (0,self.number_steps):
	    new_target=self.compute_data(itr)
	    self.robot_library.data_sendupdate(self.joint_name,new_target)
	    self.rate.sleep()

	rospy.sleep(self.SLEEP_TIME)


        return



    def compute_data(self,itr):
	pass

	return


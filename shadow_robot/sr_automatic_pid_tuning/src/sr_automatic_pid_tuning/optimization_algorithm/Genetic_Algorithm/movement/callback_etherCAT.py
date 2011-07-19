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
##Date:13 Juillet 2011

import roslib; roslib.load_manifest('sr_automatic_pid_tuning')
import rospy

from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.movement.subscriber_movement import Subscriber_Movement

from sr_robot_msgs.msg import joints_data, joint, sendupdate
import signal
import os
import time
from time import gmtime, strftime



class Callback_EtherCAT (Subscriber_Movement):
    def __init__(self,joint_name):
	Subscriber_Movement.__init__(self,joint_name)
	self.pause=False
	self.joint_name=joint_name

    def callback(self,joint_data):
	"""
	Recording all mov data
	@return nothing
	"""
	message=[]


	for joint in joint_data.joints_list:
	    if joint.joint_name==self.joint_name:
		rospy.logdebug(rospy.get_name()+"[%s] : Pos = %f | Target = %f", joint.joint_name, joint.joint_position, joint.joint_target)
		self.pause=self.break_callback(self.pause)
		if self.pause==True:
		    self.time=0
		    time.sleep(1)

		else:
		    self.record_in_file(self.time,self.time_1,0,0)#pos, #target)
		    self.time+=1
		    self.time_1+=1
	return

    def break_callback(self,pause):
	"""
	Put the callback in pause
	@return nothing
	"""
	self.pause=pause
	return self.pause

    def subscriber(self):
	"""
	init//calling callback
	@return: nothing
	"""
	self.subscriber_ = rospy.Subscriber(self.topic_name, joints_data, self.callback)

	return

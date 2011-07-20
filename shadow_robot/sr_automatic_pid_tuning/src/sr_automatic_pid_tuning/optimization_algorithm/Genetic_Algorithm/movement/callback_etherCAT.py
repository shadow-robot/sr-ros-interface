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
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

import time

class Callback_EtherCAT (Subscriber_Movement):
    def __init__(self,joint_name, controller_type = "effort"):
	Subscriber_Movement.__init__(self,joint_name)
	self.pause=False
	self.joint_name=joint_name

        self.subscriber_pos_ = None
        self.subscriber_target_ = None

        self.possible_controllers = {"effort":0, "velocity":1, "position":2}
        self.current_controller = self.possible_controllers[controller_type]
        self.current_controller_name = controller_type

        self.last_pos = 0
        self.last_target = 0
        self.pos_received = False
        self.target_received = False
        self.index_joint = None

    def callback_pos_(self, msg):
	"""
	Recording all mov data
	@return nothing
	"""
        self.pause=self.break_callback(self.pause)
        if self.pause==True:
            self.time=0
            time.sleep(1)

        else:
            pos = 0.0

            if self.index_joint == None:
                for index,name in enumerate(msg.name):
                    print name, " ", self.joint_name
                    if name in self.joint_name:
                        self.index_joint = index
                        break
            if self.index_joint is None:
                rospy.logerr("Joint not found: " + self.joint_name)
                return

            if self.current_controller == self.possible_controllers["effort"]:
                pos = msg.effort[self.index_joint]
            elif self.current_controller == self.possible_controllers["velocity"]:
                pos = msg.velocity[self.index_joint]
            elif self.current_controller == self.possible_controllers["position"]:
                pos = msg.position[self.index_joint]

            if self.target_received:
                self.record_in_file(self.time,self.time_1,pos, self.last_target)
                self.time+=1
                self.time_1+=1

                self.pos_received = False
                self.target_received = False
            else:
                self.last_pos = 0.0
                self.pos_received = True

    def callback_target_(self, msg):
	"""
	Recording all mov data
	@return nothing
	"""
        self.pause=self.break_callback(self.pause)
        if self.pause==True:
            self.time=0
            time.sleep(1)
        else:
            print "target received"

            if self.pos_received:
                self.record_in_file(self.time,self.time_1, self.last_pos, msg.data)
                self.time+=1
                self.time_1+=1

                self.pos_received = False
                self.target_received = False
            else:
                self.last_target = msg.data
                self.target_received = True

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

        target_topic_name = "/sh_"+self.joint_name.lower()+"_"+self.current_controller_name + "_controller/command/data"

        print "subscribing to : ", target_topic_name

	self.subscriber_target_  = rospy.Subscriber(target_topic_name, Float64, self.callback_target_)
        self.subscriber_pos_     = rospy.Subscriber("/joint_states", JointState, self.callback_pos_)

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
##Date:14 Juillet 2011

import roslib; roslib.load_manifest('sr_automatic_pid_tuning')
import rospy

from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.push_robot_tree.push_on_robot_tree_crack import Push_On_Robot_Tree_Crack
from sr_automatic_pid_tuning.communication_with_robot.robot_lib import Robot_Lib
from sr_robot_msgs.msg import joint,sendupdate,contrlr,joints_data

class Robot_Lib_CAN(Robot_Lib):
    def __init__(self):
        Robot_Lib.__init__(self)
        self.push_to_robot_tree = None

    def init_publisher(self):
	"""
	Initialization of the Publisher on ROS
	@return nothing
	"""
	self.publisher=rospy.Publisher("srh/sendupdate", sendupdate)

	return

    def init_subscriber(self,callback):
	"""
	Initialization of the Subscriber on ROS
	@return nothing
	"""
	rospy.Subscriber("srh/shadowhand_data", joints_data, callback)
	return

    def data_sendupdate(self,joint_name,new_target):
	"""
	Senduptade on ROS
	@return nothing
	"""
	data_to_send = [ joint(joint_name=joint_name, joint_target=new_target)]
	self.publisher.publish(sendupdate(len(data_to_send), data_to_send))

	return

    def set_pid(self, joint_name, chromosome):
        if self.push_to_robot_tree == None:
            self.push_to_robot_tree = Push_On_Robot_Tree_Crack(joint_name, self)

        self.push_to_robot_tree.pushing_values(chromosome)




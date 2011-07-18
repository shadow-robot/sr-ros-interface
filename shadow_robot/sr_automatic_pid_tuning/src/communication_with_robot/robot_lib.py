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

import rospy
import roslib; roslib.load_manifest('sr_hand')
import rospy
from sr_hand.msg import joint,sendupdate,contrlr,joints_data

class Robot_Lib(object):
    def __init__(self):

	return
	
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
	
    def robot_tree_pushing(self,path_configuration_file,hexa_value,add):
	"""
	Sending values on the Robot Tree (cracked version)
	@return nothing
	"""
	fichier=open(path_configuration_file,"w")
        fichier.write(add+" "+hexa_value)
        fichier.close()
	
	return
    
	
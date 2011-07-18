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
##Date:27 Juin 2011

import roslib; roslib.load_manifest('sr_hand')
import rospy



class Publisher_Init_Node(object):
  
    def __init__(self):
	print("init node")
	
	return
	
    
    def node_initialization(self,joint_name):
        """
        Initialize the node once
        @return: nothing
        """
	rospy.init_node("publisher_movement_GA_"+joint_name)
	
        return

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

from sr_robot_msgs.msg import joint,sendupdate,contrlr,joints_data

class Robot_Lib(object):
    def __init__(self):
        #not pretty: used to stop the whole system: there's an existing
        # instance of the Robot Library in each class we want to stop.
        self.stopped = False
        self.subscriber_ = None

    def init_publisher(self):
	"""
	Initialization of the Publisher on ROS
	@return nothing
	"""
        pass

    def init_subscriber(self,callback):
	"""
	Initialization of the Subscriber on ROS
	@return nothing
	"""
        pass

    def data_sendupdate(self,joint_name,new_target):
	"""
	Senduptade on ROS
	@return nothing
	"""
        pass

    def set_pid(self, joint_name, chromosome):
        pass



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

from std_msgs.msg import Float64
from sr_robot_msgs.srv import ForceController

import math
from sr_automatic_pid_tuning.communication_with_robot.robot_lib import Robot_Lib

class Robot_Lib_EtherCAT(Robot_Lib):
    def __init__(self, max_pwm=1023, sgleftref=0, sgrightref=0, f=0, deadband= 0, sign = 0):
        Robot_Lib.__init__(self)
        self.publisher = None
        self.msg_to_send = Float64()
        self.msg_to_send.data= 0.0
        self.pid_service = None

        self.max_pwm = max_pwm
        self.sgleftref = sgleftref
        self.sgrightref = sgrightref
        self.f = f
        self.deadband = deadband
        self.sign = sign

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
	callback.subscriber()

    def data_sendupdate(self,joint_name,new_target):
	"""
	Senduptade on ROS
	@return nothing
	"""
        if self.publisher == None:
            topic = "/sh_"+ joint_name.lower() +"_effort_controller/command"
            self.publisher = rospy.Publisher(topic, Float64)

        self.msg_to_send.data = math.radians( float( new_target ) )
        self.publisher.publish(self.msg_to_send)

    def set_pid(self, joint_name, chromosome):
        if self.pid_service == None:
            service_name =  "/realtime_loop/change_force_PID_"+joint_name
            self.pid_service = rospy.ServiceProxy(service_name, ForceController)

        print "Setting the pid for ",joint_name, " : ", chromosome
        print "    -> P:",chromosome[0], " I:",chromosome[1], " D:",chromosome[2], " Imax:" ,chromosome[3]

        self.pid_service( self.max_pwm, #max_pwm
                          self.sgleftref, #sgleftref
                          self.sgrightref, #sgrightref
                          self.f, #f
                          chromosome[0], #p
                          chromosome[1], #i
                          chromosome[2], #d
                          chromosome[3], #imax
                          self.deadband, #deadband
                          self.sign ) #sign



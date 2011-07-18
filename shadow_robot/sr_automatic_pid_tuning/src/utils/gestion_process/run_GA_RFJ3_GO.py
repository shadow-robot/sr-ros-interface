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

import roslib; roslib.load_manifest('sr_hand')
import rospy
from pid_signal import PID_Signal
from publisher_init_node import Publisher_Init_Node
import sys
sys.path.append("../../")
from optimization_algorithm.Genetic_Algorithm.genetic_algorithm import Genetic_Algorithm
from optimization_algorithm.Genetic_Algorithm.movement.callback_sub import Callback_Sub
from communication_with_robot.robot_lib import Robot_Lib

def init_node():
    ##Begining of process ~ init node
    init_node_inst=Publisher_Init_Node() 
    init_node_inst.node_initialization("RFJ3")

    return
    

def get_PID_sig():
    pid_rfj3=PID_Signal()
    pid_rfj3.get_pid_signal("RFJ3")
    return


def main():
    while not rospy.is_shutdown():
	get_PID_sig() #Break//Continue//purpose
	robot_lib=Robot_Lib()
	init_node()
	callback0=Callback_Sub("RFJ3")
	##mettre ici le roslib
	robot_lib.init_subscriber(callback0.callback)
	#callback0.subscriber()
	GA_rfj3=Genetic_Algorithm(2,2,4,0.25,"RFJ3", "random",callback0)
	GA_rfj3.give_life_to_the_system()

if __name__=="__main__":
    main()


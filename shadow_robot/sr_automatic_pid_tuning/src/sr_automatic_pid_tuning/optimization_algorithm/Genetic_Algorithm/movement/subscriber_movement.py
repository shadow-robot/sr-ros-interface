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

from sr_robot_msgs.msg import joints_data, joint, sendupdate
import signal
import os
import time
from time import gmtime, strftime
import sys
from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.config.path_config_dict import Path_Config_Dict


class Subscriber_Movement(object):
    def __init__(self,joint_name):
        self.subscriber_ = None
	#############
        self.joint_name=joint_name
        #############
        #############
        ######
        ####
        ##Paths for data
        self.path_PID_sub=Path_Config_Dict.path_root + Path_Config_Dict.path_record_data['PID_SIG_SUB_path']+self.joint_name+".txt"
        self.path_break=Path_Config_Dict.path_root + Path_Config_Dict.path_record_data['boolean_break_path']+self.joint_name+".txt"
        self.path_used_by_fitness_function=Path_Config_Dict.path_root + Path_Config_Dict.path_record_data['path_used_by_fitness']+self.joint_name+".txt"
        #complete_mvt
        clock_time=strftime("%a_%d_%b_%Y_[%H-%M-%S]")
        self.path_complete_mvt=Path_Config_Dict.path_root + Path_Config_Dict.path_record_data['complete_movement_path']+self.joint_name+"_"+clock_time+".txt"
        #Node name
        self.node_name="Subscriber_GA_"+self.joint_name
        #Topic name
        self.topic_name="srh/shadowhand_data"
	##file for fitness use
	fichier_saved1=open(self.path_used_by_fitness_function,"w")
	fichier_saved1.write("")
	fichier_saved1.close()
	##Time
	self.time=0
	self.time_1=0
	self.key=True
        ##SIG record
        self.record_PID_SIG_()

        return



    def record_PID_SIG_(self):
	"""
	Record PID SIG of the SUB
	@return: nothing
	"""
	if self.key==True:
	    PID_process=os.getpid()
	    fichier=open(self.path_PID_sub,"w")
	    fichier.write("%s" %PID_process)
	    fichier.close()

	    b_break=open(self.path_break,"w")
	    b_break.write("")
	    b_break.close()

	    self.key=False


	return

    def record_in_file(self,time,time_1,position,target):
	"""
	Save time, input and output in files
	@return: nothing
	"""
	fichier_saved0=open(self.path_complete_mvt, "a")
	fichier_saved0.write("%s" %time_1)
	fichier_saved0.write(" ")
	fichier_saved0.write("%s" %position)
	fichier_saved0.write(" ")
	fichier_saved0.write("%s" %target)
	fichier_saved0.write("\n")

	fichier_saved1=open(self.path_used_by_fitness_function,"a")
	fichier_saved1.write("%s" %time)
	fichier_saved1.write(" ")
	fichier_saved1.write("%s" %position)
	fichier_saved1.write(" ")
	fichier_saved1.write("%s" %target)
	fichier_saved1.write("\n")
	fichier_saved1.close()

	return


    def callback(self,data):
	pass


    def subscriber(self):
	"""
	init subscriber
	@return:nothing
	"""

	pass

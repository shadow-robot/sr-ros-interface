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

import tempfile
import math
from push_on_robot_tree import Push_On_Robot_Tree

from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.config.hand_config_dict import Hand_Config_Dict
from sr_automatic_pid_tuning.communication_with_robot.robot_lib import Robot_Lib

class Push_On_Robot_Tree_Crack(Push_On_Robot_Tree):
    def __init__(self,joint_name, robot_lib):
        Push_On_Robot_Tree.__init__(self)
        self.joint_name=joint_name
        self.path_node_config=Hand_Config_Dict.path_node_config[self.joint_name]
        self.robot_lib = robot_lib
        return


    def pushing_values(self,chromosome):
        """
        Send the chromosome to the robot tree [P-I-D-Imax ect]
        @return: nothing
        """
        path_node_config=self.path_node_config
	temp_file_param_saved=tempfile.TemporaryFile()
        ###
        ##0X408 -> 0xD-I-P ~ SENSOR
        ###

        for x in chromosome:
            if x==chromosome[0]: #P_sensor
                p_sensor=("%04X" % (int(x) & 0xffff))
            if x==chromosome[1]: #I_sensor
		i_sensor=("%04X" % (int(x) & 0xffff))
            if x==chromosome[2]: #D_sensor
		d_sensor=("0x%04X" % (int(x) & 0xffff))

	temp_file_param_saved.write("%s" %d_sensor)
        temp_file_param_saved.write("%s" %i_sensor)
        temp_file_param_saved.write("%s" %p_sensor)
        temp_file_param_saved.seek(0)
        new_param=temp_file_param_saved.read()
        temp_file_param_saved.close()

        self.robot_tree_pushing(,new_param,add)
        #fichier=open(path_node_config,"w")
        #fichier.write("0X408 %s" %new_param)
        #fichier.close()

        ###
        ##0X409 ->0xShift-unknown-Integral_max  ~ SENSOR
        ###
	temp_file_param_saved=tempfile.TemporaryFile()
        for x in chromosome:
            if x==chromosome[3]:  #I_max
                imax_sensor=("%04X" % (int(x) & 0xffff))
            if x==chromosome[5]:  #Shift_sensor
		shift_sensor=("0x%04X" % (int(x) & 0xffff))
            if x==chromosome[7]:  #Unknow
		unknown=("%04X" % (int(x) & 0xffff))

	temp_file_param_saved.write("%s" %shift_sensor)
        temp_file_param_saved.write("%s" %unknown)
        temp_file_param_saved.write("%s" %imax_sensor)
        temp_file_param_saved.seek(0)
        new_param=temp_file_param_saved.read()
        temp_file_param_saved.close()

        add="0X409"
        self.robot_tree_pushing(path_node_config,new_param,add)
        #fichier=open(path_node_config,"w")
        #fichier.write("0X409 %s" %new_param)
        #fichier.close()


        ###
        ##0X40a ->0xOffset-DeadBand  ~ SENSOR
        ###
	temp_file_param_saved=tempfile.TemporaryFile()
        for x in chromosome:
            if x==chromosome[4]: #DeadBand_sensor
                deadband_sensor=("%04X" % (int(x) & 0xffff))
            if x==chromosome[6]:#offset_sensor
                offset_sensor=("0x%04X" % (int(x) & 0xffff))

        temp_file_param_saved.write("%s" %offset_sensor)
        temp_file_param_saved.write("%s" %deadband_sensor)
        temp_file_param_saved.seek(0)
        new_param=temp_file_param_saved.read()
        temp_file_param_saved.close()
        add="0X40a"
        self.robot_tree_pushing(path_node_config,new_param,add)

        #fichier=open(path_node_config,"w")
        #fichier.write("0X40a %s" %new_param)
        #fichier.close()

    def robot_tree_pushing(self,path_configuration_file,hexa_value,add):
	"""
	Sending values on the Robot Tree (cracked version)
	@return nothing
	"""
	fichier=open(path_configuration_file,"w")
        fichier.write(add+" "+hexa_value)
        fichier.close()


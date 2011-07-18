# -*- coding: utf-8 -*-
##!/usr/bin/env python
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
##Date:24 Juin 2011

import roslib; roslib.load_manifest('sr_automatic_pid_tuning')
import rospy

from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.config.path_config_dict import Path_Config_Dict

from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.movement.partial_movement_sinus import Partial_Movement_Sinus
from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.movement.partial_movement_big_steps import Partial_Movement_Big_Steps
from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.movement.partial_movement_small_steps import Partial_Movement_Small_Steps
from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.movement.partial_movement_slope import Partial_Movement_Slope
import pickle
import os
import signal


class Global_Movement(object):
    def __init__(self,joint_name,instance_callback, robot_lib):
        self.joint_name=joint_name
        self.inst_callback=instance_callback
        self.robot_lib = robot_lib

    def run_movement_on_robot(self):
        """
        Instanciation of partial_movements
        @return: nothing
        """
        #self.publisher_initialization_()
        movement=[Partial_Movement_Sinus(self.joint_name, self.robot_lib),
                  Partial_Movement_Big_Steps(self.joint_name, self.robot_lib),
                  Partial_Movement_Small_Steps(self.joint_name, self.robot_lib),
                  Partial_Movement_Slope(self.joint_name, self.robot_lib)]

        for mvts in movement:
            mvts.publish_the_movement()


        rospy.sleep(1)
        return


    def communication_with_subscriber_OFF(self):
        """
        Break the Sub
        @return: nothing
        """

	self.inst_callback.break_callback(True)

        return


    def communication_with_subscriber_ON(self):
        """
        Clean all_mvts_for_fitness_JOINTNAME//Continue the Sub
        @return: nothing
        """
	file_path1=Path_Config_Dict.path_record_data['path_used_by_fitness']+self.joint_name+".txt"
	fichier_saved1=open(file_path1,"w")
	fichier_saved1.write("")
	fichier_saved1.close()
	self.inst_callback.break_callback(False)
	rospy.sleep(1)

	return


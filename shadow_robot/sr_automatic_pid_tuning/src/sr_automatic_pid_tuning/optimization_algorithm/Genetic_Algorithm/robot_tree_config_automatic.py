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


from sr_automatic_pid_tuning.communication_with_robot.robot_lib import Robot_Lib
from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.movement.global_movement import Global_Movement
from sr_automatic_pid_tuning.fitness_function.fitness_function_GA import Fitness_Function_GA

class Robot_Tree_Config_Automatic(object):
    def __init__(self,genome,joint_name,instance_callback, robot_lib):
	self.genome=genome
	self.joint_name=joint_name
        self.robot_lib = robot_lib

	self.fitness_global=Fitness_Function_GA(joint_name)
	self.instance_callback=instance_callback
	self.fitness_score=[]

	return

    def communication_with_robot_tree_(self):
	"""
	In a loop, will send each chromosome of the actual genome
	@return: nothing
	"""
	for chromosome in self.genome:
            self.robot_lib.set_pid(self.joint_name, chromosome)
	    self.fit_vect=self.movement_communication_with_sub_()

	return self.fit_vect

    def movement_communication_with_sub_(self):
        """
        Fellow the movement// com with sub// score //com with sub
        @return: the fitness vector in construction
        """
        compute_the_movement=Global_Movement(self.joint_name,self.instance_callback, self.robot_lib)
        compute_the_movement.run_movement_on_robot()
        compute_the_movement.communication_with_subscriber_OFF()
        if not self.robot_lib.stopped:
            fitness_vector_partial=self.fitness_global.complete_fitness_vector(self.fitness_score)
            self.fitness_score=fitness_vector_partial
            compute_the_movement.communication_with_subscriber_ON()

        return self.fitness_score

    def get_fitness_vector(self):
        """
        Calling private functions
        @return: the complete fitness_vector obtained after an entire genome
        """
        self.fitness_vector=self.communication_with_robot_tree_()

        return self.fitness_vector

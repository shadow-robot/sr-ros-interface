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
##Date:9 Juin 2011

import roslib; roslib.load_manifest('sr_automatic_pid_tuning')
import rospy

from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.genome import Genome
from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.selection_by_tournament import Selection_By_Tournament
from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.selection_by_elitism import Selection_By_Elitism
from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.robot_tree_config_automatic import Robot_Tree_Config_Automatic
from sr_automatic_pid_tuning.utils.gestion_process import pid_signal
from sr_automatic_pid_tuning.utils.record_actual_data import Record_Actual_Data
from sr_automatic_pid_tuning.utils.record_last_data import Record_Last_Data
from sr_automatic_pid_tuning.utils.second_type_convergence import Second_Type_Convergence
from sr_automatic_pid_tuning.utils.display.display_rxplot_service import Display_RXPLOT_Service
from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.config.path_config_dict import Path_Config_Dict
###
import time
from time import gmtime, strftime
import os
import signal
##

class Genetic_Algorithm(object):

    def __init__(self,populationSize, numberOfGenerations,
                 numberOfGenesAffectedByMutation, percentageOfMutation,
                 jointName, init_genome_type, instance_callback,
                 robot_lib  ):
        """
        """
        self.robot_lib = robot_lib
	self.instance_callback=instance_callback
            ##population size must be pair
        if populationSize%2!=0:
            self.populationSize=populationSize+1
        else:
            self.populationSize=populationSize

        self.chromosome=["P_sensor","I_sensor","D_sensor","Imax_sensor","DeadBand_sensor","Shift_sensor","Offset_sensor",0]
        self.numberOfGenerations=numberOfGenerations
        self.alien_genes=numberOfGenesAffectedByMutation
        self.percentageOfMutation=percentageOfMutation
        self.joint_name=jointName
        ##Sub SIG
        fichier_path=Path_Config_Dict.path_root + Path_Config_Dict.path_record_data['PID_SIG_SUB_path']+self.joint_name+".txt"
        fichier=open(fichier_path,"r")
	line=fichier.readline()
	fichier.close()
	self.PID=int(line)
	##
        ##first genome
        self.answer=init_genome_type
        self.genome=Genome(self.populationSize,self.alien_genes,self.percentageOfMutation,self.chromosome,self.answer)
        self.genome.choose_first_genome()
        self.first_genome=self.genome.get_actual_genome()
        ###Clock
        self.clock_time=strftime("%a_%d_%b_%Y_[%H-%M-%S]")
        print("TIME:",self.clock_time)
        ##
	###Convergences
	self.mean_fit_in_list=[]
	##Rxplot
	self.show_rxplot=Display_RXPLOT_Service(self.joint_name)
	self.rxplot_vect=[]
	##
	self.pass_to_kill=False
	self.limit_first_convergence=0.8

        self.fitness_vect0     = None
        self.fitness_vect_next = None

        return


    def selection_(self, genome, fitness_vect):
        """
        Compute the all selection
        @return: genome of best parents
        """
        ##SELECTION
        self.selection_tournament=Selection_By_Tournament(genome,self.populationSize,self.chromosome,fitness_vect)
        self.best_genome,self.winners_sc,self.winners_r=self.selection_tournament.get_best_parents()
        self.selection_elitism=Selection_By_Elitism(self.best_genome,self.populationSize,self.chromosome,fitness_vect,self.winners_sc,self.winners_r)
        self.parents_after_selection=self.selection_elitism.get_best_parents()

        return self.parents_after_selection



    def give_life_to_the_system(self):
        """
        BREEDING
        @return nothing
        """
        k=0
        generation_max=self.numberOfGenerations
        pass_to_kill=self.pass_to_kill
        first_genome_is_the_best=False

        while k<generation_max and pass_to_kill==False:
            if rospy.is_shutdown():
                self.robot_lib.stopped = True

            if self.robot_lib.stopped:
                break

            if k==0:                            #First genome
                self.genome_GA=self.first_genome
                #self.fitness_vect0=self.send_the_genome_in_robot_tree_(self.genome_GA)
                self.fitness_vect0=self.push_the_genome_on_robot_tree(self.genome_GA)
                ##Rxplot use
                if self.fitness_vect0 == None:
                    continue

                rxplot_vect=self.rxplot_use_(self.fitness_vect0)
                #test of convergence [first genome]
                pass_to_kill=self.other_convergence_(self.fitness_vect0)
                #recording other data
                record_actual_data0=Record_Actual_Data(self.fitness_vect0,self.first_genome,self.joint_name,self.clock_time,self.populationSize)
                record_actual_data0.record_all_data()
                ##
                if pass_to_kill==False:
		  self.best_parents0=self.selection_(self.genome_GA,self.fitness_vect0)
		  self.genome_next=self.genome.get_offspring(self.best_parents0)
		else:
		  first_genome_is_the_best=True

                k+=1
            else:                               #Offsprings
                self.genome_GA=self.genome_next
                print("new offspring",self.genome_next)
                #self.fitness_vect_next=self.send_the_genome_in_robot_tree_(self.genome_GA)
                self.fitness_vect_next=self.push_the_genome_on_robot_tree(self.genome_GA)
                if self.fitness_vect_next == None:
                    continue

                ##Rxplot use
                rxplot_vect=self.rxplot_use_(self.fitness_vect_next)
                #test of convergences
                pass_to_kill=self.other_convergence_(self.fitness_vect_next)
                ##recording other data
                record_actual_data0=Record_Actual_Data(self.fitness_vect_next,self.genome_next,self.joint_name,self.clock_time,self.populationSize)
		record_actual_data0.record_all_data()

                if pass_to_kill==False:
		    self.best_parents_next=self.selection_(self.genome_GA,self.fitness_vect_next)
		    self.genome_next=self.genome.get_offspring(self.best_parents_next)


                k+=1

        ##Basic convergence // Maximum number of generations
        basic_convergence_end = None
        if first_genome_is_the_best==True:
            if self.fitness_vect0 != None:
                basic_convergence_end=Record_Last_Data(self.fitness_vect0,self.first_genome,self.joint_name,self.clock_time)
	else:
            if self.fitness_vect_next != None:
                basic_convergence_end=Record_Last_Data(self.fitness_vect_next,self.genome_GA,self.joint_name,self.clock_time)
        if basic_convergence_end != None:
            basic_convergence_end.record_data_in_file()
        self.stop_system()

        return

    def push_the_genome_on_robot_tree(self,genome):
	"""
	calling functions to push on the robot tree
	@return: fitness vector
	"""
	self.matrix=genome
	robot_tree_config=Robot_Tree_Config_Automatic(self.matrix,self.joint_name, self.instance_callback, self.robot_lib)
	self.fit_vect=robot_tree_config.get_fitness_vector()

	return self.fit_vect


    def other_convergence_(self,fitness_vector):
        """
        Test of convergence
        @return the right to kill or not
        """
        ##First convergence //really high score
        first_convergence=False
        if fitness_vector != None:
            if max(fitness_vector)>=self.limit_first_convergence:
                print("STOP://FIRST CONVERGENCE//")
                first_convergence=True
                self.kill=True


            else:
                ##FALSE==> continue the breeding//CONTINUE THE GA!
                self.kill=self.pass_to_kill

            ##Second convergence //plateau
            if first_convergence==False:
                second_type_convergence=Second_Type_Convergence(fitness_vector,self.populationSize,self.mean_fit_in_list)
                self.kill=second_type_convergence.second_type_record_data()

            return self.kill


    def rxplot_use_(self,fitness_vect):
	"""
	Simple computation for rxplot use // instanciation and sending data
	@return: nothing
	"""
        if fitness_vect != None:
            fit_mean=sum(fitness_vect)/len(fitness_vect)
            self.rxplot_vect.append(fit_mean)
            self.show_rxplot.send_data_on_rxplot(self.rxplot_vect)
	return


    def stop_system(self):
        """
        Killing the sub // stop the record //cleaning pickle file (Joint_Name: "PID_SIG")
        @return: nothing
        """
        self.robot_lib.stopped = True

	#print("THE END")
	#os.kill(self.PID,signal.SIGSTOP)

        return



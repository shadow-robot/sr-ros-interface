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
##Date:5 Juillet 2011


import roslib; roslib.load_manifest('sr_automatic_pid_tuning')
import rospy

from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.config.path_config_dict import Path_Config_Dict

class Record_Actual_Data(object):
    def __init__(self,fitness_vector,offspring,joint_name,clock_time,populationSize):

        self.fitness_vector=fitness_vector
        self.populationSize=populationSize
        self.clock_time=clock_time
        self.joint_name=joint_name
        self.offspring=offspring

        ###########paths
        self.display_fitness_use_path=Path_Config_Dict.path_root + Path_Config_Dict.path_record_data['display_fit_use_path']+self.joint_name+"_"+self.clock_time+".txt"
        self.offline_use_generations_path=Path_Config_Dict.path_root + Path_Config_Dict.path_record_data['generations_used_offline_path']+self.joint_name+"_"+self.clock_time+".txt"
        self.current_best_chromosome_path=Path_Config_Dict.path_root + Path_Config_Dict.path_record_data['best_chromosome_path']+self.joint_name+"_"+self.clock_time+".txt"
        return


    def record_display_fitness_use_(self):
        """
        Record in a file the mean of each genome during the G.A
        Offline use for display
        @return: nothing
        """

        ##Data to save
        mean_fit_genomes=0
        fit_chrom=self.fitness_vector
        popu_size=self.populationSize
        mean_fit_genomes=sum(fit_chrom)/popu_size
        ###
	fichier=open(self.display_fitness_use_path,"a")
	fichier.write("%s" %mean_fit_genomes)
	fichier.write("\n")
	fichier.close()

	return

    def record_successive_generations_(self):
        """
        Record in a file the successive Genomes' generations
        Offline use if the user wants to begin the GA by one of those file
        @return: nothing
        """
        offspring=self.offspring
        data_saved_file1=open(self.offline_use_generations_path, 'a')
        data_saved_file1.write("%s \n \n" %offspring)
        data_saved_file1.close()

        return

    def record_current_best_chromosome_of_one_generation_(self):
        """
        Record in a file the successive best chromosomes of each generation
        Offline use for user
        @return: nothing
        """

        ##Data to save
        best_fit=max(self.fitness_vector)
        best_chrom_line=self.fitness_vector.index(best_fit)
        best_chrom=self.offspring[best_chrom_line]
        ##
        debug_file_best_chrom=open(self.current_best_chromosome_path, 'a')
        debug_file_best_chrom.write("best current chromosome:")
        debug_file_best_chrom.write("\n")
        debug_file_best_chrom.write("%s" %best_chrom)
        debug_file_best_chrom.write(" ")
        debug_file_best_chrom.write("%s" %best_fit)
        debug_file_best_chrom.write(" ")
        debug_file_best_chrom.write("\n")
        debug_file_best_chrom.write("\n")
        debug_file_best_chrom.close()

        return

    def record_all_data(self):
        """
        Record all in once
        @return: nothing
        """
        self.record_display_fitness_use_()
        self.record_successive_generations_()
        self.record_current_best_chromosome_of_one_generation_()

        return



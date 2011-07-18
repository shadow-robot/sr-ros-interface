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


class Record_Last_Data(object):
    def __init__(self,last_fitness_vector,offspring,joint_name,clock_time):

        #Last fitness scores
        self.last_fitness_vector=last_fitness_vector
        self.best_fitness_score=max(self.last_fitness_vector)
        #Last offspring
        self.offspring=offspring
        #One of the best chromosomes of the last offspring
        best_chromosome_raw=self.last_fitness_vector.index(self.best_fitness_score)
        self.best_chromosome=self.offspring[best_chromosome_raw]
        ##path record
        self.last_result_path=Path_Config_Dict.path_root + Path_Config_Dict.path_record_data['last_data_path']+joint_name+"_"+clock_time+".txt"
        return

    def record_data_in_file(self):
        """
        write in file the saved data
        @return: nothing
        """
        offspring=self.offspring
        last_fit=self.last_fitness_vector
        best_chrom=self.best_chromosome
        best_fit=self.best_fitness_score


        last_file=open(self.last_result_path, 'a')
        last_file.write("Last Genome:")
        last_file.write("\n")
        last_file.write("%s" %offspring)
        last_file.write("\n")
        last_file.write("\n")
        last_file.write("Fitness value for last chromosomes:")
        last_file.write("\n")
        last_file.write("%s" %last_fit)
        last_file.write("\n")
        last_file.write("\n")
        last_file.write("One of the best chromosomes:")
        last_file.write("\n")
        last_file.write("%s" %best_chrom)
        last_file.write(" ")
        last_file.write("%s" %best_fit)

        last_file.close()

        return


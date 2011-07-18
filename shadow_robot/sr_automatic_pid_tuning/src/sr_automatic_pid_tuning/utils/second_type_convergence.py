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
##Date:10 Juillet 2011

class Second_Type_Convergence(object):
    def __init__(self,last_fitness_vector,populationSize,fitness_mean_list):
        self.last_fitness_vector=last_fitness_vector
        self.populationSize=populationSize
        self.mean_fit_in_list=fitness_mean_list
        self.eps=0.02 		
        self.loop_conv=5  ##5 here, test
	self.kill_pass=False
        return
    

    def get_mean_fit_genomes_(self):
        """
        Special work before recording for second type convergence
        @return mean of the fitness vector [linked to one generation]
        """
        mean_fit_genomes=0
        fit_chrom=self.last_fitness_vector
        popu_size=self.populationSize
        self.mean_fit_genomes=sum(fit_chrom)/popu_size

        return self.mean_fit_genomes


    def second_type_record_data(self):
        """
        special work for second type convergence
        @return: right to kill or not
        """
        ##nbr of generations
	m_f_g=self.get_mean_fit_genomes_()
	self.mean_fit_in_list.append(m_f_g)
        nbr_generations_actual=len(self.mean_fit_in_list)
	
        if nbr_generations_actual%self.loop_conv==0:
            last_point=nbr_generations_actual-1
	    last_amp=self.mean_fit_in_list[last_point]
    
            ##Comparison
	    diff_1=abs(self.mean_fit_in_list[last_point]-self.mean_fit_in_list[last_point-1])
	    diff_2=abs(self.mean_fit_in_list[last_point-1]-self.mean_fit_in_list[last_point-2])

            ##Verification // the system won't stop with this convergence if it seems that the point reached is interessting
	    hierarchy=[]
		
	    for h in self.mean_fit_in_list:
		hierarchy.append(h)
		hierarchy.sort()
		hierarchy.reverse()
		
	    #key for record
	    if last_amp==hierarchy[0] or last_amp==hierarchy[1]:
		key_pass=False
		    
	    else:
		key_pass=True

            if diff_1<self.eps and diff_2<self.eps and key_pass==True:
                self.record_data_in_file()
                self.kill_pass=True
                print("STOP//PLATEAU//")

            else:
                ##will look for another plateau at the next generation
                self.mean_fit_in_list.remove(self.mean_fit_in_list[0])

        else:
            self.kill_pass=False
            
        return self.kill_pass
		

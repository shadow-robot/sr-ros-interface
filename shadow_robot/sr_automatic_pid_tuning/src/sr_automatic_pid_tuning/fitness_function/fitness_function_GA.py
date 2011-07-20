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
##Date:8 Juin 2011


import roslib; roslib.load_manifest('sr_automatic_pid_tuning')
import rospy

from sr_automatic_pid_tuning.fitness_function.partial_fitnesses.partial_fitness_MSE import Partial_Fitness_MSE
from sr_automatic_pid_tuning.fitness_function.partial_fitnesses.partial_fitness_oscillations import Partial_Fitness_Oscillations
from sr_automatic_pid_tuning.fitness_function.partial_fitnesses.partial_fitness_overshoot import Partial_Fitness_Overshoot
from sr_automatic_pid_tuning.fitness_function.partial_fitnesses.partial_fitness_risetime import Partial_Fitness_RiseTime
from sr_automatic_pid_tuning.fitness_function.partial_fitnesses.partial_fitness_settlingtime import Partial_Fitness_SettlingTime
from sr_automatic_pid_tuning.utils.get_data.get_nbr_targets import Get_Nbr_Targets
from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.config.configuration import Fitness_Config
from sr_automatic_pid_tuning.utils.get_data.get_data_from_file_GA import Get_Data_From_File_GA


class Partial_Fitness_And_Coeff(object):
    def __init__(self, partial_fitness_class=None, coeff=1):
        """
        Make a relation btw partial fitness classes and their coeff
        @return nothing
        """
        self.partial_fitness_class=partial_fitness_class
        self.coeff=coeff

        return

class Fitness_Function_GA(object):
    def __init__(self,joint_name):
        self.partial_scores=list()
        self.joint_name=joint_name
        self.partial_fitnesses_dico={}
        self.partial_fitnesses_dico['mse']=Partial_Fitness_And_Coeff(partial_fitness_class=Partial_Fitness_MSE())
        self.partial_fitnesses_dico['oscillations']=Partial_Fitness_And_Coeff(partial_fitness_class=Partial_Fitness_Oscillations())
        self.partial_fitnesses_dico['overshoot']=Partial_Fitness_And_Coeff(partial_fitness_class=Partial_Fitness_Overshoot())
        self.partial_fitnesses_dico['risetime']=Partial_Fitness_And_Coeff(partial_fitness_class=Partial_Fitness_RiseTime())
        self.partial_fitnesses_dico['settlingtime']=Partial_Fitness_And_Coeff(partial_fitness_class=Partial_Fitness_SettlingTime())
        #add the rest: all the partial fitnesses

        #Coeff values attributions linked to partial fitnesses associated
        for name in self.partial_fitnesses_dico.keys():
            self.partial_fitnesses_dico[name].coeff=Fitness_Config.coeff_dic[name]

    def weighted_mean_(self):
        """
        Computation of the partial weigted mean
        @return a vector of partial score weigted means
        """
        partial_weighted_mean=[]
        taille_caract=[]
        ##RELAX…
        for caract in self.partial_scores:
	    taille_caract.append(len(caract))

	for idx in range(0,min(taille_caract)):

            weighted_score=0
            N=0

            for pf_coeff, partial_sc in zip(self.partial_fitnesses_dico.values(),self.partial_scores):
                weighted_score+=partial_sc[idx]*pf_coeff.coeff
                N+=pf_coeff.coeff
            partial_weighted_mean.append(weighted_score/N)

        self.fit_part=partial_weighted_mean
        return self.fit_part

    def get_global_fitness_score_(self):
        """
        Computation of the final fitness score
        @return a value
        """
        self.weighted_mean_()
        fit_global=0
        n=len(self.fit_part)
        if n == 0:
            return 0.
        summ=sum(self.fit_part)
        fit_global=summ/n
        self.global_fitness_score=fit_global

        print("\n")
        print("############")
        print("global fitness score", fit_global)
        print("############")
        print("\n")
        return self.global_fitness_score


    def compute_fitness_GA_(self):
        """
        Compute the total fitness
        @return the total fitness score
        """
	self.vectors_getter=Get_Data_From_File_GA(self.joint_name)
        #get the vectors
        time_v,input_v,output_v=self.vectors_getter.get_vectors()

        if len( time_v ) == 0:
            return 0.

        self.partial_scores=[]
        #Computation of partial fitnesses scores in the same order then the coeff
        for cvs in self.partial_fitnesses_dico.values():
            self.partial_scores.append(cvs.partial_fitness_class.compute_partial_scores(time_v,input_v,output_v))


	self.one_chromosome_score=self.get_global_fitness_score_()


        return self.one_chromosome_score


    def complete_fitness_vector(self, fitness_vector_app):
        """
        Append of each chromosome's score in a vector
        @return: the fitness_vector partial that will be complete at the end of the hand config loop
        """
        one_chromosome_score=self.compute_fitness_GA_()
        fitness_vector_app.append(one_chromosome_score)
        self.fitness_vector_partial=fitness_vector_app

        return self.fitness_vector_partial





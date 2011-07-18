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

import sys
import numpy as np
import random as rand

from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.selection import Selection

class Selection_By_Tournament(Selection):

    def __init__(self,genome,populationSize,chromosome,fitness_vect):
        Selection.__init__(self)
        self.genome_for_tournament=genome
        self.populationSize=populationSize
        self.chromosome=chromosome
        self.fitness_vect=fitness_vect

        return


    def creation_of_partners_(self):
        """
        Choose randomly the pairs of chromosomes that will fight
        @return a list of couples
        """
        twosome_list=[]
        spy_twosome=[]
        rg_list=[]
        popu_size=self.populationSize
        rg_vect=range(0,popu_size)
        rg_list.extend(rg_vect)
        genome=self.genome_for_tournament
        for i in rg_vect:

            choice_one=i
            rg_list.remove(i)
            choice_two=rand.choice(rg_list)

            couple_lines=[choice_one, choice_two]

            spy_twosome.append(couple_lines)
            choice_one=genome[choice_one]
            choice_two=genome[choice_two]
            couple=[choice_one,choice_two]
            twosome_list.append(couple)

            rg_list.insert(i,i)

        self.spy_twosome=spy_twosome
        return self.spy_twosome


    def compute_tournament_(self):
        """
        Compute the tournament between chromosome of the actual genome
        @return: a list of winners' raws numbers
        """

        genome_fight=self.genome_for_tournament
        spy_twosome=self.creation_of_partners_()
        fitness_vect_copy=[]
        fitness_vect=self.fitness_vect
        boule_cristal=[0,1,1,1,1,1,1,1,1,0] #80% chance of winning
        vainqueur_lignes_list=[]
        vainqueur_fit_list=[]

        for c in fitness_vect:
            fitness_vect_copy.append(c)
        k=0
        for i in spy_twosome[:]:
            fit_vu=list()
            ligne_vu=list()
            DONE=False
            for j in spy_twosome[k][:]:
                fit_vu.append(fitness_vect[j])
                ligne_vu.append(j)

            stronger=max(fit_vu)
            weaker=min(fit_vu)

            roulette=rand.choice(boule_cristal)
            if roulette==1:
                vainqueur_fit_list.append(stronger)
                flag_fort=True
            else:
                vainqueur_fit_list.append(weaker)
                flag_fort=False

            if flag_fort==True:

                while DONE==False:
                    #print("stronger", stronger)
                    #print(" pas de stronger ou weaker dans la list?", fitness_vect_copy)
                    if fitness_vect_copy.index(stronger) in ligne_vu:
                        vainqueur_lignes_list.append(fitness_vect_copy.index(stronger))
                        DONE=True
                        fitness_vect_copy=list()
                        for c in fitness_vect:
                            fitness_vect_copy.append(c)
                    else:
                        fitness_vect_copy.insert(fitness_vect_copy.index(stronger),"rm")
                        fitness_vect_copy.remove(stronger)

            else:
                 while DONE==False:
                    #print("weaker", weaker)
                    #print(" pas de stronger ou weaker dans la list?", fitness_vect_copy)
                    if fitness_vect_copy.index(weaker) in ligne_vu:
                        vainqueur_lignes_list.append(fitness_vect_copy.index(weaker))
                        DONE=True
                        fitness_vect_copy=list()
                        for c in fitness_vect:
                            fitness_vect_copy.append(c)
                    else:
                        fitness_vect_copy.insert(fitness_vect_copy.index(weaker),"rm")
                        fitness_vect_copy.remove(weaker)


            k+=1

        self.vainqueur_lignes_list=vainqueur_lignes_list
        #print("winners after tournament",self.vainqueur_lignes_list)
        self.vainqueur_fit_list=vainqueur_fit_list
        return self.vainqueur_lignes_list, self.vainqueur_fit_list


    def reconstruct_genome_after_selection_(self):
        """
        Will use the list of winners to reconstruct the genome
        @return: a genome (matrix) and the list of the tournament winners
        """
        ##calling function
        self.compute_tournament_()
        winners_r=self.vainqueur_lignes_list
        winners_sc=self.vainqueur_fit_list
        genome=self.genome_for_tournament
        popu_size=self.populationSize
        len_chrom=len(self.chromosome)
        winners_population=np.zeros((popu_size,len_chrom))

        raw=0
        for j_spy in winners_r:
            col=0
            for m in genome[j_spy]:
                winners_population[raw][col]=m
                col+=1

            raw+=1


        self.genome_after_selection=winners_population
        self.winners_sc=winners_sc
        self.winners_r=winners_r

        return self.genome_after_selection, self.winners_sc, self.winners_r


    def get_best_parents(self):
        """
        Getter of compute_tournament
        @return: a genome (matrix) and last winners
        """
        ##calling function
        self.reconstruct_genome_after_selection_()
        self.best_parents=self.genome_after_selection
        self.last_winners_sc=self.winners_sc
        self.last_winners_r=self.winners_r
        #print("genome after tournament",self.best_parents)

        return self.best_parents, self.last_winners_sc,self.last_winners_r


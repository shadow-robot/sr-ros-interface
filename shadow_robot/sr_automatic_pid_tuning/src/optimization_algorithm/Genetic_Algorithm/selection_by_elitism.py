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

from selection import Selection
import numpy as np
import random as rand

class Selection_By_Elitism(Selection):
    
    def __init__(self,genome,populationSize,chromosome,fitness_vect,winners_sc,winners_r):
        Selection.__init__(self)
        self.genome_for_elitism=genome
        self.populationSize=populationSize
        self.chromosome=chromosome
        self.fitness_vect=fitness_vect
        self.winners_r=winners_r
        self.winners_sc=winners_sc

        return

    def compute_elitism_(self):
        """
        Compute the elitism (meaningfull only after the tournament)
        Used as a compensation of a precedent selection method
        @return: a list of winners' raws numbers after elitism
        """
        
        nbr_elites=3   # if you loose all your (nbr_elites) best chromosomes, elitism will be computed
        u=0
        popu_size=self.populationSize
        fitness_vect=[]
        for elements in self.fitness_vect:
	    fitness_vect.append(elements)
        winners_r=self.winners_r
        winners_sc=self.winners_sc
        flag_elitism=False
        
        if popu_size>=nbr_elites:
	  elite=max(fitness_vect)
	  elite_rank=fitness_vect.index(elite)
	  
	  fitness_vect.sort() 
	  fitness_vect.reverse() 
	  
	  comp1=fitness_vect[u] and fitness_vect[u+1] in winners_sc
	  #print("les trois meilleurs sont ils dans les winners?", fitness_vect[u],fitness_vect[u+1], fitness_vect[u+2])
          #print("les winners scores",winners_sc)
	  if comp1 and fitness_vect[u+2] in winners_sc:      
	      pass
	  else:
              #print("ELITISM")
              flag_elitism=True
              elite_place_random=rand.choice(winners_r)
              winners_r.remove(elite_place_random)
              winners_r.insert(elite_place_random, elite_rank)     

        self.vainqueur_lignes_list=winners_r
        self.flag_elitism=flag_elitism
        return self.vainqueur_lignes_list, self.flag_elitism 


    def reconstruct_genome_after_selection_(self):
        """
        Will use the list of winners to reconstruct the genome
        @return: a genome (matrix)
        """
        ##calling function
        self.compute_elitism_()
        flag_elitism=self.flag_elitism
        winners_r=self.vainqueur_lignes_list
        genome=self.genome_for_elitism
        popu_size=self.populationSize
        len_chrom=len(self.chromosome)
        winners_population=np.zeros((popu_size,len_chrom))
        if flag_elitism==True:
            raw=0
            for j_spy in winners_r:      
                col=0
                for m in genome[j_spy]:                 
                    winners_population[raw][col]=m      
                    col+=1

                raw+=1
            self.genome_after_selection=winners_population

        else:
            self.genome_after_selection=genome
        
        
        return self.genome_after_selection

    
    def get_best_parents(self):
        """
        Getter of compute_elitism
        @return: a genome (matrix)
        """
        ##calling function
        self.reconstruct_genome_after_selection_()
        self.best_parents=self.genome_after_selection
        #print("genome after elitism",self.best_parents)
        return self.best_parents

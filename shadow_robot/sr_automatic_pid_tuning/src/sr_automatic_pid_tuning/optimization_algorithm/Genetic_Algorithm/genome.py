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



import numpy as np
import random as rand
from sr_automatic_pid_tuning.optimization_algorithm.Genetic_Algorithm.config.configuration import Fitness_Config
import math


class Genome(object):

    def __init__(self,populationSize, parameters_set,
                 numberOfGenesAffectedByMutation,
                 percentageOfMutation, chromosome,
                 init_genome_type):

        self.parameters_set = parameters_set
        self.populationSize=populationSize
        self.percentageOfMutation=percentageOfMutation
        self.alien_genes=numberOfGenesAffectedByMutation
        self.chromosome=chromosome
        ##Protection against absurd choices on number of aliene genes
        if self.alien_genes>4:              #Only 4 genes can be affected by mutation
            print "WARNING: Too many alien genes specified: ", self.alien_genes, " instead of 4"
            self.alien_genes=4
        if self.alien_genes<0:
            print("WARNING: wrong number of alien genes")
            self.alien_genes=0
            ######
        self.percentageOfMutation=percentageOfMutation #mutation's rate
        self.genome=np.zeros((self.populationSize,len(self.chromosome)))
        self.answer=init_genome_type

        return

    def get_ranges_for_mutation_(self):
        """
        Using the Configuration file to get parameters' ranges for the process
        @return:values
        """
        ##ranges for syring of mutation
        self.syringe_min=Fitness_Config.syringe['syringe_val_min']
        self.syringe_max=Fitness_Config.syringe['syringe_val_max']

        return self.syringe_min, self.syringe_max


    def choose_first_genome(self):
        """
        Call the right computation genome creation regarding the user's choice
        @return: nothing
        """
        print ("you choose:", self.answer)
        if self.answer == "random":
            print("you choose to use a random population (genome)")
            self.computation_first_genome_random_()

        if self.answer == "saved":
            print("Using saved population")
            self.filename=raw_input("Give the file name (its path) (EX: ../../utils/saved_data_box/generation_test.txt):")
            self.computation_first_genome_saved_(self.filename)

        if self.answer != "random" and self.answer!= "saved":
            print("ERROR: your answer is uncorrect ==> random population choosen")
            self.computation_first_genome_random_()

        return


    def computation_first_genome_random_(self):
        """
        Creation of the first genome based on user's choice [RANDOM CHOICE]
        @return: a matrix from genome
        """

        ##construction of the matrix
        popu_size=self.populationSize
        len_chrom=len(self.chromosome)
        col=0

        vector_of_ranges=[]
        vector_of_ranges=[["P_sensor",self.parameters_set['P_min'],self.parameters_set['P_max']],
                          ["I_sensor",self.parameters_set['I_min'],self.parameters_set['I_max']],
                          ["D_sensor",self.parameters_set['D_min'],self.parameters_set['D_max']],
                          ["Imax_sensor",self.parameters_set['Imax_min'],self.parameters_set['Imax_max']],
                          ["DeadBand_sensor",Fitness_Config.coeff_dic['DB']],
                          ["Shift_sensor",Fitness_Config.coeff_dic['Shift']],
                          ["Offset_sensor",Fitness_Config.coeff_dic['Offset']]]

        while col<len_chrom-1:

            for chrom_param in vector_of_ranges:
                for raw in range(0,popu_size):
                    col=self.chromosome.index(chrom_param[0])
                    if len(chrom_param)==3:
                        self.genome[raw][col]=rand.randint(chrom_param[1],chrom_param[2])
                    if len(chrom_param)==2:
                        self.genome[raw][col]=chrom_param[1]
                col+=1


        print("first random genome", self.genome)


        return self.genome


    def computation_first_genome_saved_(self,filename):
        """
        Creation of the first genome based on user's choice [READ LAST GENOME FROM A FILE]
        @return: a matrix from genome
        """
        popuSize=self.populationSize
        nb_param=len(self.chromosome)
        temoin=0
        nbr_raw=0
        nbr_col=0
        autorisation=True
        key_loop=True
        kill_tst=False

        for lines in open(filename):
            test=list()
            lines = lines.strip('\n')
            lines = lines.replace('[', '')
            lines = lines.replace(']', '')
            lines = lines.split(' ')

            for i in lines:
                test.append(i)


            if len(test)==2 and test[0]==test[1] and test[0]=='' and kill_tst==False:
                kill_tst=True
                if nbr_raw!=popuSize or nbr_col!=nb_param:
                    print("ERROR: you have intancied a genome with a different population size than the one you try to take from that file")
                    autorisation=False

            nbr_raw+=1


            if key_loop==True:
                key_loop=False
                for i in lines:
                    if i!='':
                        i=float(i)
                        nbr_col+=1

        if autorisation==True:

            last_saved_genome=np.zeros((popuSize,nb_param))
            start=nbr_raw-popuSize
            raw=0
            for lines in open(filename):
                lines = lines.strip('\n')
                lines = lines.replace('[', '')
                lines = lines.replace(']', '')
                lines = lines.split(' ')
                col=0

                if temoin>=start-1:
                    for k in lines:
                        if k!='':
                            k=float(k)
                            last_saved_genome[raw][col]=k

                            col+=1
                    raw+=1

                temoin+=1

            print("last genome read in this file is:", last_saved_genome)
            self.genome=last_saved_genome

        return self.genome



    def evolution_choosing_partners_(self,best_parents):
        """
        Create random raw partners for breeding
        @return: a vector of raws' numbers
        """
        popu_size=self.populationSize
        rg_list=[]
        rg_vect=range(0,popu_size)
        rg_list.extend(rg_vect)

        rg_list_2=[]
        rg_vect_2=range(0,popu_size)
        rg_list_2.extend(rg_vect_2)

        twosome_list=[]
        spy_twosome=[]

        for i in range(0,popu_size/2):
            choice_one=rand.choice(rg_list_2)
            rg_list_2.remove(choice_one)
            choice_two=rand.choice(rg_list_2)
            rg_list_2.remove(choice_two)

            couple_lines=[choice_one, choice_two]
            spy_twosome.append(couple_lines)

            choice_one=best_parents[choice_one]
            choice_two=best_parents[choice_two]

            couple=[choice_one,choice_two]
            twosome_list.append(couple)

        self.spy_twosome=spy_twosome

        return self.spy_twosome

    def crossover_(self,best_parents):
        """
        Take into account the actual genome and compute the crossover through each chromosome's genes
        @return a matrix of chromosomes after crossover
        """
        self.evolution_choosing_partners_(best_parents)
        spy_twosome=self.spy_twosome
        popu_size=self.populationSize
        len_chrom=len(self.chromosome)
        multi_cross=range(0,3) # P-I-D => 3
        block=range(3,len_chrom)
        child_pur=np.zeros((1,len_chrom))
        child_mutated=np.zeros((1,len_chrom))
        cmpt=0
        raw=0
        genome_after_crossover=np.zeros((popu_size,len_chrom))


        for j in spy_twosome:

            parent_one_line=spy_twosome[cmpt][0]
            parent_two_line=spy_twosome[cmpt][1]

            cmpt+=1

            for i in multi_cross:

                if i%2==0:
                    choice_pur=parent_one_line
                    choice_mut=parent_two_line
                else:
                    choice_pur=parent_two_line
                    choice_mut=parent_one_line

                child_pur[0][i]=best_parents[choice_pur][i]
                child_mutated[0][i]=best_parents[choice_mut][i]
                dice=rand.randint(0,1)
                if dice==0:
                    for blk in block:
                        child_pur[0][blk]=best_parents[parent_one_line][blk]
                        child_mutated[0][blk]=best_parents[parent_two_line][blk]
                else:
                    for blk in block:
                        child_pur[0][blk]=best_parents[parent_two_line][blk]
                        child_mutated[0][blk]=best_parents[parent_one_line][blk]

            ##Matrix construction
            genome_after_crossover[raw]=child_pur
            raw+=1
            genome_after_crossover[raw]=child_mutated
            raw+=1

        #print("genome after crossover", genome_after_crossover)

        self.genome_after_crossover=genome_after_crossover
        return self.genome_after_crossover

    def crossover_and_mutation_(self, best_parents):
        """
        Mutation computation
        @return: a genome affected by mutation
        """
        popu_size=self.populationSize

        ##Preparation of the syringe
            ##Calling function
        self.get_ranges_for_mutation_()
            ####
        value_min_inject=self.syringe_min
        value_max_inject=self.syringe_max
        nbr_alien_genes=self.alien_genes
        len_chrom=len(self.chromosome)
        syringe=list()
        range_list_injected_value=range(value_min_inject,value_max_inject+1)
        max_sampler=len(range_list_injected_value)
        kill_while=False
	if nbr_alien_genes<=max_sampler:
	    syringe=rand.sample(range_list_injected_value, nbr_alien_genes)
	else:
	    syringe=rand.sample(range_list_injected_value,max_sampler)
	    rest=nbr_alien_genes-max_sampler
	    while kill_while!=True:
		for i in range(0,rest):
		    syringe.append(rand.randrange(value_min_inject,value_max_inject+1))
		kill_while=True


        self.syringe=syringe
        #print("syringe [global mutation]", syringe)
        ############################

        ##MUTATION
            ##Calling function
        self.crossover_(best_parents)
        genome_for_mutation=self.genome_after_crossover
            ####

        ####
        len_syringe=len(syringe)
        tx_mut=self.percentageOfMutation
        nbr_mutations=math.ceil((popu_size/2)*tx_mut)
        rg_list=range(0,popu_size)
        ################
        rand_chrom_mut=list()
        compt_mut=0
        while compt_mut<=nbr_mutations:

            #only 4 genes can be affected by mutation, other are fixed during the all process
            rc_list=list()
            rc_vect=range(0,len_chrom)
            rc_list.extend(rc_vect)
            rc_list.remove(Fitness_Config.secure_of_genome_and_chromosome['column_of_DeadBand_sensor'])
            rc_list.remove(Fitness_Config.secure_of_genome_and_chromosome['column_of_Shift_sensor'])
            rc_list.remove(Fitness_Config.secure_of_genome_and_chromosome['column_of_Offset_sensor'])
            rc_list.remove(Fitness_Config.secure_of_genome_and_chromosome['column_of_Unknown_sensor'])
            #

            noise=[-1,1]
            num=math.floor(abs(rand.gauss(0,2)))

            if num!=0:
                #print("mutation: noise")
                raw_rand_mut=rand.choice(rg_list)
                mutated_gene=genome_for_mutation[raw_rand_mut][0]+rand.choice(noise) #±1 P
                genome_for_mutation[raw_rand_mut][Fitness_Config.secure_of_genome_and_chromosome['column_of_P_sensor']]=mutated_gene
                mutated_gene=genome_for_mutation[raw_rand_mut][2]+rand.choice(noise) #±1 D
                genome_for_mutation[raw_rand_mut][Fitness_Config.secure_of_genome_and_chromosome['column_of_D_sensor']]=mutated_gene

            else:
                #print("mutation: global")
                rd=0
                raw_rand_mut=rand.choice(rg_list)
                for mk in range(0,nbr_alien_genes):
                    col_rand_mut=rand.choice(rc_list)
                    rc_list.remove(col_rand_mut)                                    #prevent from repeat an injection on the same gene more than once
                    mutated_gene=genome_for_mutation[raw_rand_mut][col_rand_mut]+syringe[rd]
                    genome_for_mutation[raw_rand_mut][col_rand_mut]=mutated_gene
                    rd+=1
            compt_mut+=1

        self.genome_after_crossover_and_mutation=genome_for_mutation
        #print("genome after mutation", self.genome_after_crossover_and_mutation)
        ##########################
        ##Calling function
        self.security_genome_(self.genome_after_crossover_and_mutation)
        #
        self.genome=self.genome_secure

        return self.genome


    def security_genome_(self, genome_after_crossover_and_mutation):
        """
        Mutation can lead to wrong values' signs on the genes. This function secures the genome.
        @return: the genome after crossover and mutation… after secure.
        """
        #Secure of P-I-D and Imax only
        popu_size=self.populationSize
        range_popuSize=range(0,popu_size)

        ##Security Imax (always positive)

        for raw in range_popuSize:
            if genome_after_crossover_and_mutation[raw][Fitness_Config.secure_of_genome_and_chromosome['column_of_Imax_sensor']]<0:
                genome_after_crossover_and_mutation[raw][Fitness_Config.secure_of_genome_and_chromosome['column_of_Imax_sensor']]=Fitness_Config.secure_of_genome_and_chromosome['Imax_sensor_secure']
                #print("security Imax")

        ##Security PID
            ##Mistake in the HARDWARE [D doesn't have the same sign than other]
        if Fitness_Config.secure_of_genome_and_chromosome['P-I_positive']!='True' and Fitness_Config.secure_of_genome_and_chromosome['P-I_positive']!='False':
            print(" What you choose in the Fitness Config file regarding -secure_of_genome- is INCORRECT //the process will consider that P and I are positive")
            Fitness_Config.secure_of_genome_and_chromosome['P-I_positive']='True'

        if Fitness_Config.secure_of_genome_and_chromosome['P-I_positive']=='True':
            #P-I Positive. D Negative
            for raw in range_popuSize:
                if genome_after_crossover_and_mutation[raw][Fitness_Config.secure_of_genome_and_chromosome['column_of_P_sensor']]<0:
                    genome_after_crossover_and_mutation[raw][Fitness_Config.secure_of_genome_and_chromosome['column_of_P_sensor']]=Fitness_Config.secure_of_genome_and_chromosome['P_sensor_positive_secure']
                    #print("security P")
                if genome_after_crossover_and_mutation[raw][Fitness_Config.secure_of_genome_and_chromosome['column_of_I_sensor']]<0:
                    genome_after_crossover_and_mutation[raw][Fitness_Config.secure_of_genome_and_chromosome['column_of_I_sensor']]=Fitness_Config.secure_of_genome_and_chromosome['I_sensor_positive_secure']
                    #print("security I")
                if genome_after_crossover_and_mutation[raw][Fitness_Config.secure_of_genome_and_chromosome['column_of_D_sensor']]>0:
                    genome_after_crossover_and_mutation[raw][Fitness_Config.secure_of_genome_and_chromosome['column_of_I_sensor']]=Fitness_Config.secure_of_genome_and_chromosome['D_sensor_negative_secure']
                    #print("security D")

        if Fitness_Config.secure_of_genome_and_chromosome['P-I_positive']=='False':
            #P-I Negative. D Positive
            for raw in range_popuSize:
                if genome_after_crossover_and_mutation[raw][Fitness_Config.secure_of_genome_and_chromosome['column_of_P_sensor']]>0:
                    genome_after_crossover_and_mutation[raw][Fitness_Config.secure_of_genome_and_chromosome['column_of_P_sensor']]=Fitness_Config.secure_of_genome_and_chromosome['P_sensor_negative_secure']
                    #print("security P")
                if genome_after_crossover_and_mutation[raw][Fitness_Config.secure_of_genome_and_chromosome['column_of_I_sensor']]>0:
                    genome_after_crossover_and_mutation[raw][Fitness_Config.secure_of_genome_and_chromosome['column_of_I_sensor']]=Fitness_Config.secure_of_genome_and_chromosome['I_sensor_negative_secure']
                    #print("security I")
                if genome_after_crossover_and_mutation[raw][Fitness_Config.secure_of_genome_and_chromosome['column_of_D_sensor']]<0:
                    genome_after_crossover_and_mutation[raw][Fitness_Config.secure_of_genome_and_chromosome['column_of_I_sensor']]=Fitness_Config.secure_of_genome_and_chromosome['D_sensor_positive_secure']
                    #print("security D")


        self.genome_secure=genome_after_crossover_and_mutation

        return self.genome_secure



    def get_offspring(self,best_parents):
        """
        Computation of the new genome for next loop
        @return: a matrix from precedent genome
        """
        ##Calling functions
        self.crossover_and_mutation_(best_parents)
        self.offspring=self.genome
        #print("new offspring",self.offspring)


        return self.offspring

    def get_actual_genome(self):
        """
        Return the actual genome
        @return: a matrix
        """

        return self.genome

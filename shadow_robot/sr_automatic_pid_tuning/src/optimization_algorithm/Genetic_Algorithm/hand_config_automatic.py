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
##Date:23 Juin 2011

from config.hand_config_dict import Hand_Config_Dict
import tempfile
import math
from movement.global_movement import Global_Movement
from fitness_function.fitness_function_GA import Fitness_Function_GA

class Hand_Config_Automatic(object):

    def __init__(self,genome,joint_name,instance_callback):
	self.instance_callback=instance_callback
	##For real fitness use
	self.fitness_global=Fitness_Function_GA(joint_name)
	self.fitness_score=[]
	###
        self.matrix=genome
        self.joint_name=joint_name
        ##craking config file of the robot…no concatenation possible
        self.path_node_config=Hand_Config_Dict.path_node_config[self.joint_name]
        
        return
    

    def send_config(self):
        path_node_config=self.path_node_config
        matrix=self.matrix
        len_matrix_line=len(matrix)
        range_matrix_line=range(len_matrix_line) 
        col=len(matrix[0])
        range_col=range(0,col) 
        fitness_score=self.fitness_score
        
        for i in range_matrix_line:
         
            #Range for Hexadecimal values
            range_max=max(matrix[i])+1
            range_min=min(matrix[i])-1

       
            ###
            ##0X408 -> 0xD-I-P ~ SENSOR
            ###
            
            
            for j in range_col:

                temp_file_param_saved=tempfile.TemporaryFile()
                
                for x in range(range_min, range_max):
                    
                    ##config file

                    if x==matrix[i][j]:
                        
                        if j==0:        #P_sensor [place if self.chromosome of algogen]
                            p_sensor=("%04X" % (x & 0xffff))
                            
                            
                        if j==1:        #I_sensor
                            i_sensor=("%04X" % (x & 0xffff))
                            
                            
                        if j==2:        #D_sensor
                            d_sensor=("0x%04X" % (x & 0xffff))
                            
            #print("p,i,d", p_sensor,i_sensor,d_sensor)
               
            temp_file_param_saved.write("%s" %d_sensor)
            temp_file_param_saved.write("%s" %i_sensor)
            temp_file_param_saved.write("%s" %p_sensor)
            temp_file_param_saved.seek(0)
            new_param=temp_file_param_saved.read()  
            
            temp_file_param_saved.close()   
                
                
            fichier=open(path_node_config,"w")           
            fichier.write("0X408 %s" %new_param)
            fichier.close()

            ###
            ##0X409 ->0xShift-unknown-Integral_max  ~ SENSOR
            ###



            
            for j in range_col:

                temp_file_param_saved=tempfile.TemporaryFile()

                for x in range(range_min, range_max):
              
                    ##config file

                    if x==matrix[i][j]:
                        
                        if j==3:        #I_max [place if self.chromosome of algogen]
                            imax_sensor=("%04X" % (x & 0xffff))
                            
                            
                        if j==5:        #Shift_sensor
                            shift_sensor=("0x%04X" % (x & 0xffff))
                            
                            
                        if j==7:        #Unknow : 0 ou autre qu'importe, aucune importance
                            unknown=("%04X" % (x & 0xffff))
                          

            #print("Imax,Shift", imax_sensor,shift_sensor)
                        
            temp_file_param_saved.write("%s" %shift_sensor)
            temp_file_param_saved.write("%s" %unknown)
            temp_file_param_saved.write("%s" %imax_sensor)
            temp_file_param_saved.seek(0)
            new_param=temp_file_param_saved.read()  
            temp_file_param_saved.close()   
       
            fichier=open(path_node_config,"w")
            fichier.write("0X409 %s" %new_param)
            fichier.close()
       

            ###
            ##0X40a ->0xOffset-DeadBand  ~ SENSOR
            ###



            
            for j in range_col:

                temp_file_param_saved=tempfile.TemporaryFile()

                for x in range(range_min, range_max):
              
                    ##config file

                    if x==matrix[i][j]:
                        
                        if j==4:        #DeadBand_sensor
                            deadband_sensor=("%04X" % (x & 0xffff))
                           
                            
                        if j==6:        #offset_sensor
                            offset_sensor=("0x%04X" % (x & 0xffff))
                             
            #print("deadband, offset", deadband_sensor, offset_sensor)                   
            temp_file_param_saved.write("%s" %offset_sensor)
            temp_file_param_saved.write("%s" %deadband_sensor)
            temp_file_param_saved.seek(0)
            new_param=temp_file_param_saved.read()  
            temp_file_param_saved.close()  
              
            fichier=open(path_node_config,"w")
            fichier.write("0X40a %s" %new_param)
            fichier.close()


            ###HERE, ONE chromosome has been sent to the robot tree: It's time to compute the movement
            compute_the_movement=Global_Movement(self.joint_name,self.instance_callback)
            compute_the_movement.run_movement_on_robot()
            ###Movement done: now break the sub
            compute_the_movement.communication_with_subscriber_OFF()
            ###
            ###Take all mvts for fitness into account (with real fitness function only)
            fitness_vector_partial=self.fitness_global.complete_fitness_vector(fitness_score)
            fitness_score=fitness_vector_partial
	    ###
            ###Now run the sub again
            compute_the_movement.communication_with_subscriber_ON()
            
        self.fitness_vector=fitness_score
                    
        return self.fitness_vector

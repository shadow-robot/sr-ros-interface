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
##Date:7 Juin 2011

from get_data_from_file_GA import Get_Data_From_File_GA

class Get_Nbr_Targets(Get_Data_From_File_GA):
    def __init__(self):
        Get_Data_From_File_GA.__init__(self)
        return

    def get_nbr_targets(self):
        """
        Computation of the number of targets
        @return a value (the number of targets on all the movement)
        """
        k=0
        data_input=self.input
        U=data_input[k]
        U1=data_input[k+1]
        kill1=False
        kill2=False
        data_input_virt=list()
        fast_real=list()
        ldata=len(data_input)
        aa=0
        LOW=0
        FAST=0
       
        while aa<ldata:
                data_input_virt.append(data_input[aa])
                aa+=1
        data_input_virt.append("fin")
        
        while k<len(data_input) and kill2==False:
            flat=False

            while U==U1 and kill1==False:
                flat=True
                
                if k==len(data_input)-1:
                    kill1=True
                    
                else:
                    U=data_input_virt[k]
                    U1=data_input_virt[k+1]
                    k+=1
                     
            if flat==True:   
                flat=False
                self.nbr_targets+=1
                LOW+=1
                
           
            if k==len(data_input_virt)-1:
                    kill2=True

            else:
                if (data_input_virt[k]!=data_input_virt[k+1] and data_input_virt[k]!=data_input_virt[k-1]):  #if SPEED
                    self.nbr_targets+=1
                    FAST+=1
                    U=data_input_virt[k]
                    U1=data_input_virt[k+1]
                    fast_real.append(U)
                    k+=1
                    
                else:
                    U=data_input_virt[k]
                    U1=data_input_virt[k+1]
                    k+=1

        nbr_targets=self.nbr_targets
        print("nbr of targets for this movement:",nbr_targets)
        return self.nbr_targets            
      


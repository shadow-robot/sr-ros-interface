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


from partial_fitnesses import Partial_Fitnesses
import math
import sys
sys.path.append('../../')
from utils.get_data.get_data_from_file_GA import Get_Data_From_File_GA
import numpy as np



class Partial_Fitness_SettlingTime(Partial_Fitnesses):
    def __init__(self):
        Partial_Fitnesses.__init__(self)
        self.partial_score=list()
        return

    def get_settlingtime_by_target_(self,t,x,y):
        """
        Computation of the settling time for each target
        @return a vector of solutions
        """
        virt_x=list()
        virt_y=list()
        virt_t=list()
        self.resp_time=list()
        lx=len(x)
        aa=0
        k=1
        
        while aa<lx:
            virt_x.append(x[aa])
            virt_y.append(y[aa])
            virt_t.append(t[aa])
            aa+=1

        virt_x.append("fin")
        virt_y.append("fin")
        virt_t.append(lx+1)
        
        while k<len(virt_x)-1:
            killer=False
            speed=False
            
            if virt_x[k-1]!=virt_x[k]:              
                U0=virt_x[k-1]                      
                U=virt_x[k]                         
                U1=virt_x[k+1]                      
                U_pls_25=U+0.025*U                   
                U_mns_25=U-0.025*U                  
                t0=virt_t[k-1]                      
                ty=virt_t[k]                        
                out=False

                while out==False and U1==U :         

                    if ty==len(x)-1:           
                        out=True
                        
                    else:
                        U1=x[int(ty+1)]        
                        U=x[int(ty)]           
                        U0=x[int(ty-1)]
                        ty+=1

                if U!=U1 and U!=U0:            
                    U1=virt_x[int(ty+1)]        
                    U=virt_x[int(ty)]           
                    U0=virt_x[int(ty-1)]
                    ty=virt_t[k]
                    speed=True

                if y[int(ty)]<=U_pls_25 and y[int(ty)]>=U_mns_25: 
                    ty-=1

                    while y[int(ty)]<=U_pls_25 and y[int(ty)]>=U_mns_25 and killer==False and speed==False:
                        ty-=1
                        if ty<=t0:
                            self.resp_time.append(-1)
                            killer=True                               

                    if (y[int(ty)]>U_pls_25 or y[int(ty)]< U_mns_25) and (killer==False) and speed==False:
                            self.resp_time.append((ty+1)-t0)        

                    if speed==True:
                        t0=t[k-1]
                        tact=t[k]
                        val_tps=abs(t0-tact)
                        self.resp_time.append(val_tps)
           
                else:
                    self.resp_time.append(-1)

            k+=1
            
        settlingtime=self.resp_time
        #print("\n")
        #print("settling time by target", settlingtime)
        return self.resp_time


    def compute_partial_scores(self,time_v,input_v,output_v):
        """
        Compute the partial score corresponding to the settling time of each target
        @return: a vector a partial fitness scores
        """
        self.get_settlingtime_by_target_(time_v,input_v,output_v)
        vect_rt=self.resp_time
        rt_rw=list()
        k=0
                                                                
        while k<len(vect_rt):                                  
            x=vect_rt[k]
            
            if x==-1:                                           
                rt_rw.append(0)
                k+=1
                
            else:
                f_rt=(np.exp(-x)/(np.exp(-x*0.1)))**0.01
                
                if math.isinf(f_rt)==True or math.isnan(f_rt) or f_rt>1:
                    f_rt=1
                rt_rw.append(f_rt)
                k+=1

        self.partial_score=rt_rw
        #print("\n")
        #print("settling time partial score", rt_rw)
        return self.partial_score

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


class Partial_Fitness_RiseTime(Partial_Fitnesses):
    def __init__(self):
        Partial_Fitnesses.__init__(self)
        self.partial_score=list()
        return

    def get_risetime_by_target_(self,t,x,y):
        """
        Computation of the rising time for each target
        @return a vector of solutions
        """   
        self.rise_time=list()
        virt_x=list()                  
        virt_y=list()
        virt_t=list()
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
        
        while k<len(x):
            speed=False
          
            if virt_x[k]!=virt_x[k+1] and virt_x[k]!=virt_x[k-1]:
                speed=True
                t0=virt_t[k-1]
                tact=virt_t[k]
                val_tps=abs(t0-tact)
                self.rise_time.append(val_tps)
  
            if x[k-1]!=x[k] and speed==False:              
                U0=x[k-1]                      
                U=x[k]                         
                delta_U=U-U0
                save_k=k
                b=0

                            
                if delta_U>0:
                    U_10=U0+delta_U*0.1                          
                    U_90=U0+delta_U*0.9                          
                    asc=True
                   
                else:
                    U_90=U0+delta_U*0.9
                    U_10=U0+delta_U*0.1
                    asc=False

                while b<len(x)-save_k:         
                                                                                                     
                    if (y[save_k]>=U_10 and y[save_k]<U_90) and asc==True:     
                        compt=0
                        ta=t[save_k]
                        save_k+=1                                               
                        
                        while compt<len(x):                        
                            
                            if save_k<len(x) and y[save_k]>=U_90:
                                tb=t[save_k]
                                self.rise_time.append(abs(tb-ta))
                                compt=len(x)-save_k                 
                                b=len(x)-save_k                    
                                break
                            
                            else:                                       
                                compt+=1
                                save_k+=1
                                if save_k+1<len(x) and x[save_k]!=x[save_k+1] or save_k+1==len(x):    
                                    compt=len(x)-save_k-1
                                    self.rise_time.append(-1)
                                    break

                        break
                   
                    if (y[save_k]>U_90 and y[save_k]<=U_10) and asc==False:   
                        compt=0
                        ta=t[save_k]
                        save_k+=1
                               
                        while compt<len(x):
                            
                            if save_k<len(x) and y[save_k]<=U_90:
                                tb=t[save_k]
                                self.rise_time.append(abs(tb-ta))
                                compt=len(x)-save_k                
                                b=len(x)-save_k                    
                                break
                            
                            else:                                       
                                compt+=1
                                save_k+=1
                                
                                if save_k+1<len(x) and x[save_k]!=x[save_k+1] or save_k+1==len(x):    
                                    compt=len(x)-save_k-1
                                    self.rise_time.append(-1)
                                    break
                        break                                                                      
        
                    
                    else:
                        save_k+=1
              
                        if save_k<len(x) and x[save_k]!=x[save_k-1] or b==len(x)-save_k:                              
                            b=len(x)-save_k-1
                            self.rise_time.append(-1)
                            break
                        else:
                            b+=1
                            
                        
                k+=1
                
            else:
                k+=1

        risetime=self.rise_time
        ##print("\n")
        ##print("rise time by target", risetime)
        return self.rise_time

    def compute_partial_scores(self,time_v,input_v,output_v):
        """
        Compute the partial score corresponding to the rising time of each target
        @return: a vector a partial fitness scores
        """
        self.get_risetime_by_target_(time_v,input_v,output_v)
        vect_tm=self.rise_time
        tm_rw=list()
        k=0
        
        while k<len(vect_tm):                                   
            x=vect_tm[k]                                       
            

            if x==-1:                                           
                tm_rw.append(0)
                k+=1
                
            else:
                f_tm=(np.exp(-x)/(0.01+np.exp(-(x*0.1))))**0.025
                
                if math.isinf(f_tm)==True or math.isnan(f_tm) or f_tm>1:
                    f_tm=1
                tm_rw.append(f_tm)
                k+=1

        self.partial_score=tm_rw
        ##print("\n")
        ##print("rise time partial score",tm_rw)
        return self.partial_score

    

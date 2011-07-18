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


from partial_fitnesses import Partial_Fitnesses
import math

import numpy as np



class Partial_Fitness_MSE(Partial_Fitnesses):
    def __init__(self):
        Partial_Fitnesses.__init__(self)
        self.partial_score=list()
        return

    def mse_global_computation_(self,time_vect,input_vect,output_vect):
        """
        Computation of the mse for all the movement
        @return the global mse value
        """
        len_vect=len(time_vect)
        mean_square_error=0
        nb_measures=0

        for i in range(0,len_vect):
            mean_square_error+=(float(input_vect[i])-float(output_vect[i]))**2
            nb_measures+=1

        mean_square_error/=nb_measures
        self.global_mse=mean_square_error
        ##print("\n")
        ##print("global MSE", mean_square_error)
        return self.global_mse

    def mse_by_target_(self,time_v,input_v,output_v):
        """
        Computation of the mse target by target
        @return a vector containing each result
        """
        k=1
        mse_cmds=list()
        t=time_v
        y=output_v
        x=input_v

        virt_x=list()
        virt_y=list()
        virt_t=list()

        kc=0
        lx=len(x)
        aa=0
        while aa<lx:
            virt_x.append(x[aa])
            virt_y.append(y[aa])
            virt_t.append(t[aa])
            aa+=1

        virt_x.append("fin")
        virt_y.append("fin")
        virt_t.append(lx+1)

        kill=False

        flats=False


        if virt_x[kc]!=virt_x[kc+1]:
            clc_mse=0
            nbr_fst=0
            clc_mse+=(virt_y[kc]-virt_x[kc])**2
            nbr_fst+=1

            clc_mse/=nbr_fst
            mse_cmds.append(clc_mse)

        while k<len(t)-1 and kill==False:

            nbr_ms_cmds=0
            clc_mse=0

            while kill==False and (virt_x[k]!=virt_x[k+1] and virt_x[k]!=virt_x[k-1]):

                if k<=len(x):

                    clc_mse+=(virt_y[k]-virt_x[k])**2
                    nbr_ms_cmds+=1

                    clc_mse/=nbr_ms_cmds
                    mse_cmds.append(clc_mse)
                    k+=1

                if k>=len(virt_x)-1:
                    kill=True


            while kill==False and virt_x[k]==virt_x[k-1]:

                if k<len(virt_t)-1:

                    clc_mse+=(virt_y[k]-virt_x[k])**2
                    nbr_ms_cmds+=1
                    k+=1
                    flats=True


                if k>=len(virt_t)-1:
                    kill=True

            if nbr_ms_cmds!=0 and flats==True:

                clc_mse/=nbr_ms_cmds
                mse_cmds.append(clc_mse)

                k-=1
                flats=False

            k+=1

        self.mse_by_target=mse_cmds
        ##print("\n")
        ##print("mse by target", mse_cmds)
        return self.mse_by_target
    
    def compute_partial_scores(self, time_v, input_v, output_v):
        """
        Compute the partial score corresponding to the mse for each target
        @return: a vector a partial fitness scores
        """
        #Calling function#
        self.mse_by_target_(time_v,input_v,output_v)

        vect_mse=self.mse_by_target
        mse_rw=list()

        k=1

        #Use of partial fitness function f_mse#
        while k<len(vect_mse):
            x=vect_mse[k]
            f_mse=(np.exp(-(0.8*x-5))/(1+np.exp(-((x**0.1)*0.1-5))))**0.1
            if math.isinf(f_mse)==True or math.isnan(f_mse) or f_mse>1:
                f_over=1

            mse_rw.append(f_mse)
            k+=1
        ##

        self.partial_score=mse_rw
        ##print("\n")
        ##print("mse partial score",mse_rw)
        return self.partial_score

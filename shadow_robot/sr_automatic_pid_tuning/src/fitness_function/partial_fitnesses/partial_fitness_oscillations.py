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
import sys
sys.path.append('../../')
from utils.get_data.get_data_from_file_GA import Get_Data_From_File_GA
from optimization_algorithm.Genetic_Algorithm.config.configuration import Fitness_Config
import numpy as np



class Partial_Fitness_Oscillations(Partial_Fitnesses):
    def __init__(self):
        Partial_Fitnesses.__init__(self)
        self.partial_score=list()
        #Using the Configuration file 
        self.eps=Fitness_Config.coeff_dic['eps']
        self.cst_yi=Fitness_Config.coeff_dic['cst_yi']
        self.cst_moy=Fitness_Config.coeff_dic['cst_moy']
        self.cst_yk=Fitness_Config.coeff_dic['cst_yk']
        self.j=Fitness_Config.coeff_dic['j']
        return

    def nbr_oscillations_by_target_(self,t,x,y):
        """
        Computation of the number of significative oscillations on each target
        @return all results in a vector
        """

        #use of configuration file
        eps=self.eps
        cst_yi=self.cst_yi
        cst_moy=self.cst_moy
        cst_yk=self.cst_yk
        j=self.j
        #
        
        list_y=list()
        list_x=list()
        mean_main=list()
        flat=False
        p=0
        alert=0
        k=0
        i=j+k
        slope=list()
        cmds_slope=list()
        slope_tst=list()
        nbr_slp_cmds=list()
        virt_x=list()
        virt_y=list()
        virt_t=list()
        lx=len(x)
        aa=0
        compt=0
        fast_osc=list()
        FAST=0
        fastx=False
        flatx=False
        while aa<lx:
            virt_x.append(x[aa])
            virt_y.append(y[aa])
            virt_t.append(t[aa])
            aa+=1
        virt_x.append(-100)
        virt_y.append(-100)
        virt_t.append(lx+1)

        clef=False
        if virt_x[0]==virt_x[1]:
            clef=True

        clef1=False
        if virt_x[0]!=virt_x[1]:
            compt+=1
            clef1=True

        while i<len(virt_t)-1:
            if fastx==True:
                i-=1
                fastx=False

            if flatx==True:
                i-=1
                flatx=False

            U0=virt_x[i-1]
            U1=virt_x[i+1]
            U=virt_x[i]
            out=False

            if U!=U1 and U!=U0 or clef1==True:
                clef1=False
                flat_x=False
                FAST+=1
                if compt==1:
                    nbr_slp_cmds.append(0)

                U0=virt_x[i-1]
                U=virt_x[i]
                U1=virt_x[i+1]
                list_y.append(virt_y[i-1])
                list_x.append(virt_x[i-1])
                fast_osc.append(U)
                nbr_slp_cmds.append(0)
                i+=j
                flat=False

            while U==U1  and out==False or clef==True:
                go=False
                pp=0
                go2=False
                pp_list=list()
                ordre_mean=list()

                if len(list_y)!=0:
                    compt_osc_x=0
                    compt_osc_x2=0
                    while pp<len(list_y)-1:
                        if int(list_y[pp])!=int(list_y[pp+1]):
                            pp_list.append(list_y[pp])
                        pp+=1

                    if len(list_x)!=0:
                        for ks in list_x:
                            lm=0
                            main_list=list()
                            kill_it=False
                            while lm<len(list_x)-1:
                                if int(list_x[lm])==int(list_x[lm+1]) and kill_it==False:
                                    lm+=1
                                    main_list.append(list_x[lm-1])

                                else:
                                    lm+=1
                                    kill_it=True

                    if len(main_list)!=0 and len(pp_list)!=0 and len(pp_list)>=3:
                        len_main=len(main_list)
                        mean_main=(sum(main_list)/len_main)

                        for jkp in pp_list:
                            if jkp>mean_main:
                                ordre_mean.append("sup")

                            if jkp<mean_main:
                                ordre_mean.append("inf")

                        ordre_mean_virt=list()
                        rg_ord=range(len(ordre_mean))
                        for copy in rg_ord:
                            ordre_mean_virt.append(copy)
                        ordre_mean_virt.remove(ordre_mean_virt[0])
                        ordre_mean_virt.remove(ordre_mean_virt[len(ordre_mean_virt)-1])

                        for kd in ordre_mean_virt:
                            if (ordre_mean[kd-1]=="sup" and ordre_mean[kd+1]=="sup" and ordre_mean[kd]=="inf") or (ordre_mean[kd-1]=='inf' and ordre_mean[kd+1]=="inf" and ordre_mean[kd]=="sup"):
                                compt_osc_x+=3
                                #print("compt osc", compt_osc_x)
                                go=True

                            if go==True:

                                go=False
                                for ksn in range(0,len(pp_list)):
                                    nbr_slp_cmds.remove(0)
                                    nbr_slp_cmds.append(compt_osc_x)

                            if (ordre_mean[kd-1]=="sup" and ordre_mean[kd]=="inf") or (ordre_mean[kd-1]=="inf" and ordre_mean[kd]=="sup"):
                                compt_osc_x2+=2
                                go2=True

                            if go2==True:
                                go2=False
                                for ksn in range(0,len(pp_list)):
                                    nbr_slp_cmds.remove(0)
                                    nbr_slp_cmds.append(compt_osc_x2)

                list_y=list()
                pp_list=list()
                list_x=list()
                mean_main=list()
                clef=False
                compt+=1
                flat=True
                if i==len(virt_t)-1:
                    out=True

                else:
                    U0=virt_x[i-1]
                    U=virt_x[i]
                    U1=virt_x[i+1]


                    compare_yk=abs((virt_y[k])-(virt_y[i]))

                    if compare_yk>=cst_yk:
                        if abs(virt_y[i-1]-virt_y[i+1])<=eps:

                           if abs(virt_y[i]-virt_y[i+1])<=eps and abs(virt_y[i]-virt_x[i])<=eps:
                               a=virt_y[k]
                               b=virt_y[i]
                               slope.append((virt_y[k]-virt_y[i])/(virt_t[k]-virt_t[i]))
                               k=i
                               i+=j

                           else:
                               i+=j

                        else:
                            compare_yi=abs(virt_y[i]-virt_x[i])
                            moy1=(virt_y[i-1]+virt_y[i])/2
                            moy2=(virt_y[i]+virt_y[i+1])/2


                            if compare_yi>=cst_yi and abs(moy1-moy2)>=cst_moy:
                                a=virt_y[k]
                                b=virt_y[i]
                                slope.append((virt_y[k]-virt_y[i])/(virt_t[k]-virt_t[i]))
                                k=i
                                i+=j

                            else:
                                i+=j


                    else:
                        i+=j

                    if (virt_x[i-1]!=virt_x[i]) :
                        if i<len(virt_x)-1 and virt_x[i]!=virt_x[i+1] and virt_x[i]!=virt_x[i-1]:
                            fastx=True



                        if i<len(virt_x)-1 and virt_x[i]==virt_x[i+1]:
                            flatx=True

                        if alert==1:
                            m=0

                            while p<len(slope):
                                slope_tst.append(slope[p])
                                p+=1

                            while m<l0:
                                slope_tst[m]=slope[l0-1]
                                m+=1

                            cmds_slope=slope_tst
                            nbr=0
                            xx=0

                            while xx<len(cmds_slope)-1:
                                signe=cmds_slope[xx]*cmds_slope[xx+1]
                                if signe<0:
                                    nbr+=1
                                xx+=1

                            nbr_slp_cmds.append(nbr)
                            cmds_slope=list()
                            l0=len(slope)
                            i+=1

                        elif len(slope)!=0:

                            while p<len(slope):
                                slope_tst.append(slope[p])
                                p+=1

                            cmds_slope=slope_tst
                            alert+=1
                            l0=len(slope)
                            i+=1
                            nbr=0
                            xx=0
                            while xx<len(cmds_slope)-1:
                                signe=cmds_slope[xx]*cmds_slope[xx+1]
                                if signe<0:
                                    nbr+=1
                                xx+=1
                            nbr_slp_cmds.append(nbr)



                        if len(slope)==0:
                            nbr_slp_cmds.append(0)
                            i+=1



        self.nbr_oscillations=nbr_slp_cmds
        ##print("\n")
        ##print("nbr osc by target", nbr_slp_cmds)
        return self.nbr_oscillations


    def compute_partial_scores(self, time_v, input_v, output_v):
        """
        Compute the partial score corresponding to the number of oscillations for each target
        @return: a vector a partial fitness scores
        """

        vect_osc=self.nbr_oscillations_by_target_(time_v,input_v,output_v)
        #vect_osc=self.nbr_oscillations
        osc_rw=list()
        O_tolerance=False
        kill_out=False
        k=1



        while k<len(vect_osc):

            if kill_out==False:

                for zs in vect_osc:

                    if zs>=3:
                        O_tolerance=True
                        kill_out=True

            if O_tolerance==False:
                x=vect_osc[k]
                f_osc=(np.exp(-(x-2))/(0.01+np.exp(-((x**0.13)*0.1-2))))**0.3

                if math.isinf(f_osc)==True or math.isnan(f_osc) or f_osc>1:
                    f_osc=1
                osc_rw.append(f_osc)
                k+=1

            else:
                x=vect_osc[k]

                if x!=0:
                    f_osc=(np.exp(-(x-2))/(0.01+np.exp(-((x**0.13)*0.1-2))))**0.3
                    if math.isinf(f_osc)==True or math.isnan(f_osc) or f_osc>1:
                        f_osc=1
                    osc_rw.append(f_osc)
                    k+=1

                else:
                    no_tolerance=2
                    f_osc=(np.exp(-(no_tolerance-2))/(0.01+np.exp(-((no_tolerance**0.13)*0.1-2))))**0.3

                    if math.isinf(f_osc)==True or math.isnan(f_osc) or f_osc>1:
                        f_osc=1
                    osc_rw.append(f_osc)
                    k+=1

        self.partial_score=osc_rw
        ##print("\n")
        ##print("oscillations partial score", osc_rw)
        return self.partial_score

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


from sr_automatic_pid_tuning.fitness_function.partial_fitnesses.partial_fitnesses import Partial_Fitnesses
import math
import roslib; roslib.load_manifest('sr_automatic_pid_tuning')
import rospy

import numpy as np


class Partial_Fitness_Overshoot(Partial_Fitnesses):
    def __init__(self):
        Partial_Fitnesses.__init__(self)
        self.partial_score=list()
        return

    def percentage_of_overshoot_by_target_(self,t,x,y):
        """
        Compute the percentage of the first overshoot on each target
        @return a vector of each solution
        """
        self.overshoot=list()
        self.taux_overshoot=list()
        vect_U=list()
        vect_U0=list()
        vect_yinf=list()
        vect_y0=list()
        upper=False
        lower=False
        virt_x=list()
        virt_y=list()
        virt_t=list()
        lx=len(x)
        aa=0
        f=0
        eps=1
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
            if virt_x[k-1]!=virt_x[k]:

                U0=virt_x[k-1]
                U=virt_x[k]
                U1=virt_x[k+1]
                out=False
                ty=int(t[k])
                ycmd=list()
                vect_U.append(U)
                vect_U0.append(U0)

                if virt_x[int(ty)]!=virt_x[int(ty+1)] and virt_x[int(ty)]!=virt_x[int(ty-1)]:
                    speed=True
                    y0=virt_y[k-1]
                    y=virt_y[k]
                    U0=virt_x[k-1]
                    U=virt_x[k]

                    if abs(y-U)>eps:
                        val=abs(y-U)
                        self.taux_overshoot.append(val)

                    else:
                        self.taux_overshoot.append(0)

                while U1==U and out==False:
                    speed=False

                    if ty==len(x)-1:
                        out=True

                    else:
                        U1=x[int(ty+1)]
                        U=x[int(ty)]
                        ycmd.append(virt_y[ty])
                        ty+=1

                if len(ycmd)!=0:
                    vect_yinf.append(ycmd[len(ycmd)-1])
                    vect_y0.append(ycmd[0])

                if U0<U and speed==False:
                    upper=True
                    w=0
                    maxi=False
                    while w<len(ycmd):
                        val=ycmd[w]
                        if val>U:
                            maxi=True
                            point_m=w
                            break

                        else:
                            pass

                        w+=1
                    n=0
                    vect_yval=list()
                    yval=0
                    yval_0=0
                    yval_1=0
                    kill_2=False

                    if maxi==True:

                        while n<len(ycmd)-1-point_m and kill_2==False:
                            yval_0=ycmd[point_m]
                            kill_1=False
                            n+=1
                            yval=ycmd[point_m+n]

                            if yval_0<yval:

                                if point_m+n+1==len(ycmd):
                                    self.overshoot.append(yval)
                                    kill_1=True

                                else:
                                    yval_1=ycmd[point_m+n+1]

                                while kill_1==False:

                                    if yval>yval_1 or n==len(ycmd)-1-point_m:
                                        self.overshoot.append(yval)
                                        kill_1=True
                                        kill_2=True

                                    elif n!=len(ycmd)-1-point_m:
                                        yval=ycmd[point_m+n]
                                        yval_1=ycmd[point_m+n+1]
                                        n+=1

                            else:

                                self.overshoot.append(yval_0)
                                kill_2=True

                        if point_m==len(ycmd)-1:
                                self.overshoot.append(yval_0)
                                kill_1=True

                    else:
                        self.overshoot.append(0)

                if U0>U and speed==False:

                    lower=True
                    w=0
                    mini=False
                    while w<len(ycmd):
                        val=ycmd[w]
                        if val<U:
                            mini=True
                            point_m=w
                            break

                        else:
                            pass

                        w+=1
                    n=0
                    vect_yval=list()
                    yval=0
                    yval_0=0
                    yval_1=0
                    kill_2=False

                    if mini==True:

                        while n<len(ycmd)-1-point_m and kill_2==False:
                            n+=1
                            yval_0=ycmd[point_m]
                            yval=ycmd[point_m+n]
                            kill_1=False

                            if yval_0>yval:

                                if point_m+n+1==len(ycmd):
                                    self.overshoot.append(yval)
                                    kill_1=True

                                else:
                                    yval_1=ycmd[point_m+n+1]

                                while kill_1==False:
                                    if yval<yval_1 or n==len(ycmd)-1-point_m:
                                        self.overshoot.append(yval)
                                        kill_1=True
                                        kill_2=True

                                    elif n!=len(ycmd)-1-point_m:
                                        yval=ycmd[point_m+n]
                                        yval_1=ycmd[point_m+n+1]
                                        n+=1

                            else:
                                self.overshoot.append(yval_0)
                                kill_2=True

                        if point_m==len(ycmd)-1:
                            self.overshoot.append(yval_0)
                            kill_1=True

                    else:
                        self.overshoot.append(0)

                while f<len(self.overshoot) and speed==False and (upper==True or lower==True):

                    if self.overshoot[f]!=0 and vect_yinf[f]!=vect_y0[f]:
                        tx_prc2=((self.overshoot[f]-vect_yinf[f])*100)/(vect_yinf[f]-vect_y0[f])
                        self.taux_overshoot.append(tx_prc2)

                    else:
                        self.taux_overshoot.append(0)
                    f+=1

                upper=False
                lower=False

            k+=1

        percentage_overshoot=self.taux_overshoot
        #print("\n")
        #print("percentage of overshoot by target",percentage_overshoot)
        return self.taux_overshoot

    def compute_partial_scores(self,time_v,input_v,output_v):
        """
        Compute the partial score corresponding to the percentage of overshoot of each target
        @return: a vector a partial fitness scores
        """
        self.percentage_of_overshoot_by_target_(time_v,input_v,output_v)
        vect_over=self.taux_overshoot
        over_rw=list()
        k=0

        while k<len(vect_over):
            x=vect_over[k]
            f_over=(np.exp(-x)/(0.1+np.exp(-(x*0.1+0.1))))**0.1

            if math.isinf(f_over)==True or math.isnan(f_over) or f_over>1:
                f_over=1
            over_rw.append(f_over)
            k+=1

        self.partial_score=over_rw
        #print("\n")
        #print("overshoot partial score", over_rw)
        return self.partial_score

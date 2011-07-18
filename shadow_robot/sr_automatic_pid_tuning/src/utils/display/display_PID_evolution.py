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
##Date:12 Juillet 2011

import numpy as np
from numpy import matrix
import matplotlib.pyplot as plt
import os



class Display_PID_Evolution(object):
    def __init__(self):
	question=raw_input("Give the Generation filename[EX: ../saved_data_box/Generation….txt]:")
	filename=question
        nbr_col=0
        nbr_raw=0
        key_loop=True
        key_loop2=True
        self.filename=filename
        saut=list()
        for lines in open(filename):            
            test=list()
            lines = lines.strip('\n')
            lines = lines.replace('[', '')
            lines = lines.replace(']', '')
            lines = lines.split(' ')
            for i in lines:
                test.append(i)
            if len(test)==2 and test[0]==test[1] and test[0]=='':
                saut.append(nbr_raw)
                if key_loop2==True:
                    popuSize=nbr_raw
                    key_loop2=False                   
                nbr_raw-=1
            if key_loop==True:
                key_loop=False
                for i in lines:
                    if i!='':           
                        i=float(i)                        
                        nbr_col+=1          
            nbr_raw+=1
            
        self.nbr_lines=nbr_raw
        self.nbr_col=nbr_col
        self.saut=saut
        self.popuSize=popuSize
    
    def get_number_of_raws(self):
        return self.nbr_lines


    def get_number_of_col(self):
        return self.nbr_col

    def get_matrix(self):    
        r=self.nbr_lines
        c=self.nbr_col
        lignes_de_saut=self.saut
        k=0
        filename=self.filename
        huge_matrix=np.zeros((r,c))
        raw=0
        
        for lines in open(filename):
            key=True
            lines = lines.strip('\n')
            lines = lines.replace('[', '')
            lines = lines.replace(']', '') 
            lines = lines.split(' ')
            col=0
            for i in lines:

                if k<len(lignes_de_saut) and raw==lignes_de_saut[k]:
                    key=False
                    k+=1
              
                    raw-=1
                if i!='' and key==True:          
		    i=float(i)              
                    huge_matrix[raw][col]=i
                    col+=1
            raw+=1           
        self.matrix=huge_matrix
        self.generationSize=r
        self.len_chrom=c
        return self.matrix

    def P_mean(self):
        matrice=self.matrix
        generationSize=self.generationSize
        popuSize=self.popuSize
        P_type_mean=list()
        k=0
        j=0
        P_mean=list()
        P_mean_x=list()
        matrice=matrix(matrice)
        matrice=matrice.T                  
        matrice=np.array(matrice)
      
	while k<generationSize:          
            P_mean.append((np.sum(matrice[0][k:popuSize+k]))/popuSize)
            P_mean_x.append(sum(P_mean)/len(P_mean)) 
            k+=popuSize           
       
	self.p_mean=P_mean_x
	return self.p_mean    

    def I_mean(self):
        matrice=self.matrix
        generationSize=self.generationSize
        popuSize=self.popuSize
        I_mean=list()
        I_mean_x=list()
        k=0

        matrice=matrix(matrice)
        matrice=matrice.T
        matrice=np.array(matrice)

        k=0
        while k<generationSize:
            I_mean.append((np.sum(matrice[1][k:popuSize+k]))/popuSize)
            I_mean_x.append(sum(I_mean)/len(I_mean))
            k+=popuSize

        self.i_mean=I_mean_x


        return self.i_mean



    def D_mean(self):
        matrice=self.matrix
        generationSize=self.generationSize
        popuSize=self.popuSize
        
        k=0
        D_mean=list()
        D_mean_x=list()
        matrice=matrix(matrice)
        matrice=matrice.T
        matrice=np.array(matrice)

        while k<generationSize:
            D_mean.append((np.sum(matrice[2][k:popuSize+k]))/popuSize)
            D_mean_x.append(sum(D_mean)/len(D_mean))
            k+=popuSize

        self.d_mean=D_mean_x

        return self.d_mean


    def display_genes_ev(self):
        p_vect_mean=self.p_mean
        i_vect_mean=self.i_mean
        d_vect_mean=self.d_mean
        

        plt.figure(1)
        plt.plot(p_vect_mean,'b')
        plt.ylabel("Mean of P -evolution")
        plt.xlabel("Number of generation")
        
        plt.figure(2)
        plt.plot(i_vect_mean,'r')
        plt.ylabel("Mean of I -evolution")
        plt.xlabel("Number of generation")

        plt.figure(3)
        plt.plot(d_vect_mean,'g')
        plt.ylabel("Mean of D -evolution")
        plt.xlabel("Number of generation")

        plt.show()

        
        return


def main():
    show0=Display_PID_Evolution()
    show0.get_number_of_raws()
    show0.get_number_of_col()
    show0.get_matrix()
    show0.P_mean()
    show0.I_mean()
    show0.D_mean()
    show0.display_genes_ev()

    
if __name__=="__main__":
    main()


    
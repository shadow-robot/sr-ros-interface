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

import matplotlib.pyplot as plt

from matplotlib.pyplot import *


class Display_Fitness_Evolution(object):
    def __init__(self):
	fichier=raw_input("Give the display_use file path [EX: ../saved_data_box/display_fit_use….txt]:")
	fit_mean=list()
	fit_mean_x=list()
	for lines in open(fichier):
	    lines=lines.strip("\n")
	    lines=float(lines)
	    fit_mean.append(lines)
	    ##Mean	
	    fit_mean_x.append(sum(fit_mean)/len(fit_mean))    
	self.fit_mean=fit_mean_x
	self.fit_mean_simple=fit_mean
	return



  
    def display_genes_ev(self):
	fit_vect_mean=self.fit_mean
	simple_mean=self.fit_mean_simple
	p1=plot(fit_vect_mean,"m")
	p2=plot(simple_mean,"y")
	l1=legend([p1],["global evolution of the mean"], loc=4)
	l2=legend([p2],["mean of each genome"],"best")
	plt.ylabel("Fitness Score")
	plt.xlabel("Generations")
	gca().add_artist(l1)
	show()
   
	return
   
   
   
def main():
    show0=Display_Fitness_Evolution()
    show0.display_genes_ev()
   
    
if __name__=="__main__":
    main()



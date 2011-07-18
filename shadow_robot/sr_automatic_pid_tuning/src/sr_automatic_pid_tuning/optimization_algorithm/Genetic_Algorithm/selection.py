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



class Selection(object):
    def __init__(self):
        
        return

    def reconstruct_genome_after_selection_(self):
        """
        Will reconstruct the genome's matrix after the selection
        @return: a matrix
        """

        return self.genome_after_selection
    
    def get_best_parents(self):
        """
        Will return the matrix of best parents after selection methods
        @return: a matrix
        """
        return self.best_parents

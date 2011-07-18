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
##Date:6 Juin 2011


class Get_Data(object):
    def __init__(self):
        self.time=list()
        self.input=list()
        self.output=list()
        self.nbr_targets=0
        return

    def get_vectors(self):
        """
        This gives access to each vector
        @return vectors of time, input,output
        """
        #print("time",self.time)
        #print("input",self.input)
        #print("output",self.output)
        return self.time, self.input, self.output

    def get_nbr_targets(self):
        """
        Does noting
        @return the global number of targets
        """
        return self.nbr_targets

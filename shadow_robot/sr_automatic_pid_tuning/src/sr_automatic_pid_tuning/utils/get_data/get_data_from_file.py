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

from sr_automatic_pid_tuning.utils.get_data import Get_Data

class Get_Data_From_File(Get_Data):
    def __init__(self,filename):
        Get_Data.__init__(self)
        Get_Data.get_vectors(self)
        self.filename=filename
        nb_measures=0
        mean_square_error=0

        for line in open(self.filename):
            line = line.strip('\n')
            splitted = line.split(' ')
            self.time.append(float(splitted[0]))
            self.output.append(float(splitted[1]))
            self.input.append(float(splitted[2]))
        return


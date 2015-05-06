#!/usr/bin/env python
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
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#

VERBOSE = 1

class Grasp(object):

    def __init__(self):
        self.grasp_name = ""
        self.joints_and_positions = {}

    def display_grasp(self):
        print self.grasp_name
        print self.joints_and_positions

    def convert_to_xml(self):
        grasp = '	<grasp name="'+self.grasp_name+'">'+'\n'
        for key, value in self.joints_and_positions.items():
            grasp = grasp + '		<joint name="'+str(key)+'">'+str(value)+'</joint>\n'
        grasp = grasp + '	</grasp>'+'\n'
        return grasp

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

import roslib; roslib.load_manifest('sr_automatic_pid_tuning')
import rospy

class FitnessFunction(object):
    def __init__( self, robot_communication, joint_name,
                  movement):
        self.robot_comm = robot_communication
        self.joint_name = joint_name

        self.movement = movement

        self.targets = []
        self.positions = []

    def fitness_function(self, pids, additional_parameters, reduction_factor,
                         parameters_order):
        param_dict = {}
        index = 0

        for param in pids:
            param_dict[parameters_order[index]] = param * reduction_factor
            index += 1
        for param in additional_parameters:
            param_dict[parameters_order[index]] = param
            index += 1

        print param_dict
        self.robot_comm.set_pid(self.joint_name, param_dict)

        self.move_and_record()

        global_fitness = self.compute_fitness()

        #print pids, " Additional: ", additional_parameters,"  reduction:  ",reduction_factor
        return global_fitness

    def move_and_record(self):
        self.targets, self.positions = self.movement.move_and_record()

    def compute_fitness(self):
        #TODO: extend this to use different evaluations
        mean_square_error = 0
        for target,pos in self.targets,self.positions:
            mean_square_error += (target - pos)*(target - pos)
        mean_square_error /= len(target - pos)

        return mean_square_error

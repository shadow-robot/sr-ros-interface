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

import numpy, scipy
from scipy.optimize import fmin

class AutomaticPidTuner(object):
    """
    Automatic pid tuning using downhill simplex.
    """
    MAX_PARAM_CONST_ = 1000

    def __init__(self, fitness_function, joint_name,
                 starting_vector = None,
                 mask = [1,1,1,1,0,
                         0,0,0,
                         0,0],
                 parameters_order = ["p", "i", "d", "imax", "f",
                                     "max_pwm", "sgleftref", "sgrightref",
                                     "deadband", "sign"],
                 callback = None, reduction_factor = 10000):
        """
        Automatic pid tuning using downhill simplex.

        @param fitness_function: the function you want to use to compute the fitness.
        @param joint_name:       the joint on which we want to run the optimization
        @param starting_vector:  [optional] if given, a starting vector for the simplex,
                                            otherwise we use a random vector.
        @param mask:             [optional] a mask to optimize only certain parameters
        @param parameters_order: [optional] a list of the parameters names in the order they're given
                                            in the starting vector
        @param callback:         [optional] a function called at each iteration
        @param max_iter:         [optional] the maximum number of iterations
        #param reduction_factor: [optional] a factor by which we multiply the values in the
                                            fitness function, then cast them as int (the pid
                                            parameters are integers)

        @return a list containing:
           - the set of optimized parameters in a list: [P,I,D,Imax]
           - a list with more data:
              - the fitness for those parameters
              - the number of iterations performed
              - the number of function calls made
              - a warnflag: * 1 if the maximum number of function evaluations was made
                            * 2 if the maximum number of iterations was reached
        """
        if starting_vector is None:
            starting_vector = []
            for i in parameters_order:
                starting_vector.append(scipy.random.randint(self.MAX_PARAM_CONST_))

        #uses the mask to find out which values are going to be
        # optimized and which values are going to be left out.
        additional_parameters = []
        starting_vector_tmp = []
        if mask is not None:
            for param, optimize_this_param in zip(starting_vector, mask):
                if optimize_this_param:
                    #the values on which we run the simplex are divided by a reduction
                    # factor, then multiplied by the same factor when setting them as
                    # pids (in order to work with "real" numbers not ints)
                    starting_vector_tmp.append(param / reduction_factor)
                else:
                    additional_parameters.append(param)
        starting_vector = starting_vector_tmp

        self.additional_parameters = additional_parameters
        self.starting_vector = starting_vector
        self.parameters_order = parameters_order
        self.fitness_function = fitness_function
        self.callback = callback
        self.reduction_factor = reduction_factor

    def run(self):
        full_results = fmin( self.fitness_function,
                             self.starting_vector,
                             [ self.additional_parameters, self.reduction_factor,
                               self.parameters_order ],
                             callback = self.callback,
                             full_output = True,
                             ftol = 100,
                             maxiter = 100,
                             maxfun = 3000)

        optimized_pids = full_results[0] * self.reduction_factor
        optimized_pids = numpy.hstack( (optimized_pids, self.additional_parameters) )

        return optimized_pids, full_results[1:]




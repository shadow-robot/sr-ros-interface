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

from sr_automatic_pid_tuning.sr_automatic_pid_tuning import AutomaticPidTuner
from sr_automatic_pid_tuning.fitness_function.fitness_function import FitnessFunction

def main():
    """
    This is an example on how to use the automatic pid tuner.
    For more convenience, the pid tuner is integrated in the
    sr_control_gui.
    """
    fit = FitnessFunction()

    apt = AutomaticPidTuner( fit.fitness_function,
                             [10,0,0,0,0,
                              1023,0,0,
                              0,0],
                             reduction_factor = 1)
    optimized_pids, full_results = apt.run()

    print " ---- "
    print optimized_pids, "  -> ", full_results[0]


# start the script
if __name__ == "__main__":
    main()

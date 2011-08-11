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

class Movement(object):
    """
    """

    def __init__(self, robot_communication, joint_name):
        """
        """
        self.robot_communication = robot_communication
        self.joint_name = joint_name

        self.movements = []

        self.targets = []
        self.positions = []

    def move_and_record(self):
        #TODO: implement this
        for movement in self.movements:
            movement.move()

        return self.targets,self.positions



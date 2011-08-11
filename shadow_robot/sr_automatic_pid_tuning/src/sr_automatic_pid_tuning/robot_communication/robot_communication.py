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

from etherCAT_hand_lib import EtherCAT_Hand_Lib

class RobotCommunication(object):
    """
    """

    def __init__(self, ):
        """
        """
        pass

    def sendupdate(self, joint_name, target):
        """
        """
        pass

    def get_target_and_position(self, joint_name):
        """
        """
        pass

    def set_pid(self, joint_name, pid_parameters):
        """
        """
        pass



class EtherCATRobotCommunication(RobotCommunication):
    """
    """

    def __init__(self, controller_type = "effort"):
        """
        """
        RobotCommunication.__init__(self)

        self.robot_lib = EtherCAT_Hand_Lib()
        self.robot_lib.activate()
        self.controller_type = controller_type

    def sendupdate(self, joint_name, target):
        """
        """
        self.robot_lib.sendupdate(joint_name, target, self.controller_type)

    def get_position(self, joint_name):
        """
        """
        self.robot_lib.get_position(joint_name)

    def set_pid(self, joint_name, pid_parameters):
        """
        """
        self.robot_lib.set_pid(joint_name, pid_parameters)

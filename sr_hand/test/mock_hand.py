#!/usr/bin/env python

# ####################################################################
# Copyright (c) 2013 Shadow Robot Company Ltd.
# All rights reserved.
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
# ####################################################################

import rospy;
from pr2_mechanism_msgs.srv import *

class MockHand(object):
    """
    A ros node to look enough like a shadowhand for these tests.

    List controller service so it looks like a ethercat hand to HandCommander.
    """

    def __init__(self, ):
        """
        @brief Construct a new MockHand, setting up ros connections.
        """
        self.list_srv = rospy.Service('pr2_controller_manager/list_controllers',
                ListControllers, self.list_controllers_cb)
        rospy.loginfo("Started MockHand")

    def list_controllers_cb(self, req):
        res = ListControllersResponse()
        res.controllers = ['sh_ffj0_position_controller', 'sh_ffj3_position_controller', 'sh_ffj4_position_controller', 'sh_lfj0_position_controller', 'sh_lfj3_position_controller', 'sh_lfj4_position_controller', 'sh_lfj5_position_controller', 'sh_mfj0_position_controller', 'sh_mfj3_position_controller', 'sh_mfj4_position_controller', 'sh_rfj0_position_controller', 'sh_rfj3_position_controller', 'sh_rfj4_position_controller', 'sh_thj1_position_controller', 'sh_thj2_position_controller', 'sh_thj3_position_controller', 'sh_thj4_position_controller', 'sh_thj5_position_controller', 'sh_wrj1_position_controller', 'sh_wrj2_position_controller']
        res.state = ['running', 'running', 'running', 'running', 'running', 'running', 'running', 'running', 'running', 'running', 'running', 'running', 'running', 'running', 'running', 'running', 'running', 'running', 'running', 'running']
        return res


if __name__ == "__main__":
    try:
        rospy.init_node("mock_hand")
        node = MockHand()
        rospy.spin();
    except rospy.ROSInterruptException:
        pass


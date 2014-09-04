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

import rospy
from time import sleep

from controller_manager_msgs.srv import ListControllers, LoadController

class LoadAllControllers(object):
    def __init__(self):
        params = rospy.get_param("")

        unique_keys = []
        for key in params.keys():
            if ("sh" in key) and ("controller" in key):
                unique_keys.append(key)

        #making unique
        unique_keys = list(set(unique_keys))

        for controller in unique_keys:
            try:
                load_controllers = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
                resp1 = load_controllers(controller)
            except rospy.ServiceException:
                rospy.logerr("Failed to load controller for "+controller)
            rospy.loginfo("Loaded "+controller+" sucessfully.")

if __name__ == '__main__':
    rospy.init_node("load_all_controllers")
    lac = LoadAllControllers()

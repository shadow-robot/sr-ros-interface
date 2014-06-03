#!/usr/bin/python

# Copyright 2014 Shadow Robot Company Ltd.
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

import rospy
import threading

from sr_robot_msgs.msg import BiotacAll, ShadowPST, UBI0All

"""
Module that receives the tactile values.
"""

class TactileReceiver():

    def __init__(self):
        self.tactile_type = self.find_tactile_type()
        self.tactile_state = None
        
        if self.tactile_type == "PST":
            self.tactile_listener = rospy.Subscriber("tactile", ShadowPST, self.tactile_callback)
        elif self.tactile_type == "biotac":
            self.tactile_listener = rospy.Subscriber("tactile", BiotacAll, self.tactile_callback)
        elif self.tactile_type == "UBI0":
            self.tactile_listener = rospy.Subscriber("tactile", UBI0All, self.tactile_callback)
        
        
    def find_tactile_type(self):
        try:
            rospy.wait_for_message("tactile", ShadowPST, timeout = 0.2)
            return "PST"
        except:
            pass
        
        try:
            rospy.wait_for_message("tactile", BiotacAll, timeout = 0.2)
            return "biotac"
        except:
            pass
            
        try:
            rospy.wait_for_message("tactile", UBI0All, timeout = 0.2)
            return "UBI0"
        except:
            rospy.logwarn("No tactile topic found. This is normal for a simulated hand")
            
        return None
    
    def tactile_callback(self, tactile_msg):
        self.tactile_state = tactile_msg
    
    def get_tactile_type(self):
        return self.tactile_type
        
    def get_tactile_state(self):
        return self.tactile_state

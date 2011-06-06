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

import roslib; roslib.load_manifest('sr_control_gui')
import rospy

from joint_slider import Joint, JointSlider
  
class HandJointSlider(JointSlider):  
    """
    Hand joint sliders to move the hand joints with sliders.
    """
    name = "Hand Joint Sliders"
        
    def __init__(self):
        joints_list = []
        
        joints_list.append(Joint("FFJ0", 0, 180))
        joints_list.append(Joint("FFJ3"))
        joints_list.append(Joint("FFJ4", -25, 25))
        
        joints_list.append(Joint("MFJ0", 0, 180))
        joints_list.append(Joint("MFJ3"))
        joints_list.append(Joint("MFJ4", -25, 25))
        
        joints_list.append(Joint("RFJ0", 0, 180))
        joints_list.append(Joint("RFJ3"))
        joints_list.append(Joint("RFJ4", -25, 25))
        
        joints_list.append(Joint("LFJ0", 0, 180))
        joints_list.append(Joint("LFJ3"))
        joints_list.append(Joint("LFJ4", -25, 25))
        joints_list.append(Joint("LFJ5", 0, 45))
        
        joints_list.append(Joint("THJ1"))
        joints_list.append(Joint("THJ2", -30, 30))
        joints_list.append(Joint("THJ3", -15, 15))
        joints_list.append(Joint("THJ4", 0, 70))
        joints_list.append(Joint("THJ5", -60, 60))
        
        joints_list.append(Joint("WRJ1", -35, 45))
        joints_list.append(Joint("WRJ2", -30, 10))
        
        JointSlider.__init__(self, joints_list)

    def sendupdate(self, dict):
        self.parent.parent.libraries["sr_library"].sendupdate_from_dict(dict)
        self.set_icon(self.parent.parent.rootPath + '/images/icons/iconHand.png')

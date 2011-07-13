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

from joint_slider import Joint, LightJointSlider
from config import Config

from std_msgs.msg import Float64

class VelocityJointSlider(LightJointSlider):
    """
    Sliders to send a velocity demand to a motor on the etherCAT
    hand.
    """
    name = "Velocity Sliders"

    def __init__(self):
        joints_tmp = [ ["FFJ0", "/sh_ffj0_velocity_controller/command"] ,
                       ["FFJ3", "/sh_ffj3_velocity_controller/command"] ,
                       ["FFJ4", "/sh_ffj4_velocity_controller/command"] ,

                       ["MFJ0", "/sh_mfj0_velocity_controller/command"] ,
                       ["MFJ3", "/sh_mfj3_velocity_controller/command"] ,
                       ["MFJ4", "/sh_mfj4_velocity_controller/command"] ,

                       ["RFJ0", "/sh_rfj0_velocity_controller/command"] ,
                       ["RFJ3", "/sh_rfj3_velocity_controller/command"] ,
                       ["RFJ4", "/sh_rfj4_velocity_controller/command"] ,

                       ["LFJ0", "/sh_lfj0_velocity_controller/command"] ,
                       ["LFJ3", "/sh_lfj3_velocity_controller/command"] ,
                       ["LFJ4", "/sh_lfj4_velocity_controller/command"] ,
                       ["LFJ5", "/sh_lfj5_velocity_controller/command"] ,

                       ["THJ1", "/sh_thj1_velocity_controller/command"] ,
                       ["THJ2", "/sh_thj2_velocity_controller/command"] ,
                       ["THJ3", "/sh_thj3_velocity_controller/command"] ,
                       ["THJ4", "/sh_thj4_velocity_controller/command"] ,
                       ["THJ5", "/sh_thj5_velocity_controller/command"] ,

                       ["WRJ1", "/sh_wrj1_velocity_controller/command"] ,
                       ["WRJ2", "/sh_wrj2_velocity_controller/command"]   ]

        joints_list = []
        self.publishers = {}

        for j in joints_tmp:
            joints_list.append( Joint(j[0], 0, 150) )
            self.publishers[j[0]] = rospy.Publisher(j[1], Float64)

        LightJointSlider.__init__(self, joints_list)

        self.dependencies = None


    def sendupdate(self, dict):
        for item in dict.items():
            self.publishers[item[0]].publish(Float64(item[1]/100.))

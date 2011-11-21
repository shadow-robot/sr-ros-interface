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
from config import Config

class ArmJointSlider(JointSlider):
    """
    Sliders to move the arm.
    """
    name = "Arm Joint Sliders"

    def __init__(self):
        joints_list = [Joint("ShoulderJRotate", -45, 60),
                       Joint("ShoulderJSwing", 0, 80),
                       Joint("ElbowJSwing", 0, 120),
                       Joint("ElbowJRotate", -80, 80)
                       ]

        JointSlider.__init__(self, joints_list)
        self.dependencies = Config.shadow_arm_plugin_config.dependencies

    def sendupdate(self, dict):
        self.parent.parent.libraries["sr_library"].sendupdate_arm_from_dict(dict)
        self.set_icon(self.parent.parent.rootPath + '/images/icons/iconArm.png')

    def depends(self):
        return Config.shadow_arm_plugin_config.dependencies

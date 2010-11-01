#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_control_gui')
import rospy

from joint_slider import Joint, JointSlider
from config import Config

class ArmJointSlider(JointSlider):  
    name = "Arm Joint Sliders"
        
    def __init__(self):        
        joints_list = [Joint("trunk_rotation", -45, 90),
                       Joint("shoulder_rotation", 0, 90),
                       Joint("elbow_abduction", 0, 120),
                       Joint("forearm_rotation", -90, 90)
                       ]
        
        JointSlider.__init__(self, joints_list)
        
        
    def sendupdate(self, dict):
        self.parent.parent.libraries["sr_library"].sendupdate_arm_from_dict(dict)
        self.set_icon(self.parent.parent.rootPath + '/images/icons/iconArm.png')
    
    def depends(self):
        return Config.shadow_arm_plugin_config.dependencies
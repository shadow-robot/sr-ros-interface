#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_control_gui')
import rospy

from shadowhand_ros import ShadowHand_ROS

import os, sys
sys.path.append(os.getcwd() + "/plugins")
from generic_plugin import GenericPlugin

class ShadowGenericPlugin(GenericPlugin):  
    name = "Shadow Robot Generic Plugin"
    
    def __init__(self):
        GenericPlugin.__init__(self)
        
    def activate(self):
        self.sr_library = ShadowHand_ROS()
        GenericPlugin.activate(self)
    
    #def on_close(self):
    #    self.sr_library.__del__()
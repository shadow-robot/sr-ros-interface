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
        self.sr_library = None
        
    def activate(self):
        GenericPlugin.activate(self)
        # only activate the first time you open the window
        if self.sr_library !=  None:
            return
        self.sr_library = ShadowHand_ROS()
    
    def on_close(self):
        GenericPlugin.on_close(self)
        #self.sr_library.__del__()
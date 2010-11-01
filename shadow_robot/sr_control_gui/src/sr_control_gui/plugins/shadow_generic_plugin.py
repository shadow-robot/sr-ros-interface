#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_control_gui')
import rospy

from shadowhand_ros import ShadowHand_ROS
from PyQt4 import QtCore, QtGui, Qt

import os, sys

#Not very pretty....
import subprocess
process = subprocess.Popen("rospack find sr_control_gui".split(), stdout=subprocess.PIPE)
rootPath = process.communicate()[0]
rootPath = rootPath.split('\n')
rootPath = rootPath[0]
sys.path.append(rootPath+ "/src/sr_control_gui/plugins")

from generic_plugin import GenericPlugin

class ShadowGenericPlugin(GenericPlugin):  
    name = "Shadow Robot Generic Plugin"
    
    def __init__(self):
        GenericPlugin.__init__(self)
        
    def activate(self):
        GenericPlugin.activate(self)
        # only activate the library if it hasn't been already loaded
        if self.parent.parent.libraries.get("sr_library") == None:
            self.parent.emit(QtCore.SIGNAL("messageToStatusbar(QString)"), 
                             "Loading Shadowhand Library...")
            self.parent.parent.libraries["sr_library"] = ShadowHand_ROS()
            self.parent.emit(QtCore.SIGNAL("messageToStatusbar(QString)"), 
                             "Shadowhand Library Loaded.")
        
    def on_close(self):
        GenericPlugin.on_close(self)
    
    def depends(self):
        return ["Shadow Hand"]
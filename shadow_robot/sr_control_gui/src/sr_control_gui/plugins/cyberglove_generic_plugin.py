#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_control_gui')
import rospy

from cyberglove_library import Cyberglove

import os, sys

#Not very pretty....
import subprocess
process = subprocess.Popen("rospack find sr_control_gui".split(), stdout=subprocess.PIPE)
rootPath = process.communicate()[0]
rootPath = rootPath.split('\n')
rootPath = rootPath[0]
sys.path.append(rootPath+ "/src/sr_control_gui/plugins")

from generic_plugin import GenericPlugin

class CybergloveGenericPlugin(GenericPlugin):  
    name = "Cyberglove Robot Generic Plugin"
    
    def __init__(self):
        GenericPlugin.__init__(self)
        
    def activate(self):
        GenericPlugin.activate(self)
        # only activate the library if it hasn't been already loaded
        if self.parent.parent.libraries.get("cyberglove") == None:
            self.parent.parent.libraries["cyberglove"] = Cyberglove()
        
    def on_close(self):
        GenericPlugin.on_close(self)
        
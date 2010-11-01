#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_control_gui')
import rospy

from cyberglove_library import Cyberglove
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

class CybergloveGenericPlugin(GenericPlugin):  
    name = "Cyberglove Robot Generic Plugin"
    
    def __init__(self):
        GenericPlugin.__init__(self)
        
    def activate(self):
        GenericPlugin.activate(self)
        # only activate the library if it hasn't been already loaded
        if self.parent.parent.libraries.get("cyberglove") == None:
            self.parent.emit(QtCore.SIGNAL("messageToStatusbar(QString)"), 
                             "Loading Cyberglove Library...")
            try:
                self.parent.parent.libraries["cyberglove"] = Cyberglove()
            except:
                self.parent.emit(QtCore.SIGNAL("messageToStatusbar(QString)"), 
                                 "Couldn't load the Cyberglove Library.")
                self.window.close()
                
                self.parent.parent.libraries.get("cyberglove") == None
                return
            
            self.parent.emit(QtCore.SIGNAL("messageToStatusbar(QString)"), 
                             "Cyberglove Library Loaded.")
        
    def on_close(self):
        GenericPlugin.on_close(self)
        
    def depends(self):
        return ["Cyberglove"]
        
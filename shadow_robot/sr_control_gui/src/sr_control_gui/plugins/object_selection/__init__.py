#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_control_gui')
import rospy

from PyQt4 import QtCore, QtGui, Qt

from generic_plugin import GenericPlugin
from config import Config

class ObjectSelection(GenericPlugin):  
    """
    Contact the tabletop object detector to get a list of the objects on top 
    of the table. Then possibility to select a detected object for picking it up.
    """
    name = "Object Selection"
        
    def __init__(self):
        GenericPlugin.__init__(self)
    
    def activate(self):
        GenericPlugin.activate(self)
    def on_close(self):
        GenericPlugin.on_close(self)
    def depends(self):
        return Config.sr_object_selection_config.dependencies
                

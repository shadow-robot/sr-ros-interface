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

from shadowhand_ros import ShadowHand_ROS
from PyQt4 import QtCore, QtGui, Qt

from generic_plugin import GenericPlugin
from config import *

class ShadowGenericPlugin(GenericPlugin):  
    """
    The mother of all plugins using the shadow robot ROS interface (Hand and Arm).
    """
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
        return Config.shadowhand_plugin_config.dependencies

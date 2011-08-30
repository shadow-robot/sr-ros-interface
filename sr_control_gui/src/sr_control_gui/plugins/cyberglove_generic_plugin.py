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

from cyberglove_library import Cyberglove
from PyQt4 import QtCore, QtGui, Qt

from generic_plugin import GenericPlugin
from config import *

class CybergloveGenericPlugin(GenericPlugin):
    """
    The mother of all plugins using the cyberglove ROS interface.
    """
    name = "Cyberglove Robot Generic Plugin"

    def __init__(self):
        GenericPlugin.__init__(self)
        self.dependencies = Config.cyberglove_generic_plugin_config.dependencies

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


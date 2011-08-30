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

from PyQt4 import QtCore, QtGui, Qt

class GenericDockWidget(QtGui.QDockWidget):
    """
    The parent class for the dock widgets, which are docked in the MainWindow.
    """
    def __init__(self, parent):
        QtGui.QDockWidget.__init__(self, parent=parent)
        self.shown = False
        self.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        self.hide()

    def show_hide(self):
        if self.shown:
            self.hide()
        else:
            self.show()

    def hide(self):
        self.shown = False
        self.parent().show_robot_and_libraries.setText('Show Robot / Ros nodes')
        self.parent().show_robot_and_libraries.setIcon(QtGui.QIcon(self.parent().rootPath + '/images/icons/show.png'))
        QtGui.QDockWidget.hide(self)

    def show(self):
        self.shown = True
        self.parent().show_robot_and_libraries.setText('Hide Robot / Ros nodes')
        self.parent().show_robot_and_libraries.setIcon(QtGui.QIcon(self.parent().rootPath + '/images/icons/hide.png'))
        QtGui.QDockWidget.show(self)

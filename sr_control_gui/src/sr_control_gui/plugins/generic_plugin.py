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

from PyQt4 import QtCore, QtGui, Qt
from plugin_manager import Plugin
from config import *

class MyMdiSubWindow(QtGui.QMdiSubWindow):
    def __init__(self, parent):
        """
        This MdiSubWindow removes itself from the MdiArea when it closes. It can then be reopened.
        """
        self.container = 0
        self.parent = parent
        QtGui.QMdiSubWindow.__init__(self)

    def set_container(self, container):
        self.container = container

    def closeEvent(self, event):
        self.parent.on_close()
        self.container.removeSubWindow(self)
        #self.container.closeActiveSubWindow()

class GenericPlugin(Plugin):
    """
    A generic plugin from which all the plugins inherit.
    Opens a subwindow in the MainWidget.
    """
    name = "GenericPlugin"

    def __init__(self):
        Plugin.__init__(self)
        self.dependencies = Config.generic_plugin_config.dependencies
        self.id = 0
        self.parent = 0
        self.window = MyMdiSubWindow(self)
        #self.window.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        self.window.widget()
        self.window.setWindowTitle(self.name)
        self.is_window_opened = False

    def set_parent(self, parent):
        self.parent = parent
        self.window.set_container(self.parent.container)

    def activate(self):
        if not self.is_window_opened:
            self.parent.container.addSubWindow(self.window)
            self.window.show()
            self.is_window_opened = True
        else:
            self.parent.container.setActiveSubWindow(self.window)


    def set_icon(self, path):
        self.window.setWindowIcon(QtGui.QIcon(path))

    def on_close(self):
        self.is_window_opened = False
        self.window.close()

    def depends(self):
        return Config.generic_plugin_config.dependencies

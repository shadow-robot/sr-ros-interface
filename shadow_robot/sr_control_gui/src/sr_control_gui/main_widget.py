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

from plugin_manager import PluginManager
from PyQt4 import Qt, QtCore, QtGui, QtWebKit
import os, sys
from config import Config

class MainWidget(QtGui.QWidget):
    """
    The MainWidget is a QMdiArea in which the plugins will be spawned.
    """
    def __init__(self, parent):
        QtGui.QWidget.__init__(self)
        self.parent = parent
        self.layout = QtGui.QVBoxLayout()
        self.container = QtGui.QMdiArea(self)
        self.container.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        self.container.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)

        self.layout.addWidget(self.container)
        self.setLayout(self.layout)
        ####
        # LOAD AVAILABLE PLUGINS
        ##
        # Create plugin manager
        self.manager = PluginManager()

        self.manager.setPluginPlaces([self.parent.rootPath + "/src/sr_control_gui/plugins"])

        # Add the plugins to the menubar in tool menu
        tools = self.parent.menuBar().addMenu('&Plugins')
        self.parent.statusBar().showMessage('Loading plugins...', 500)

        # Load plugins
        nb_plugins_found = self.manager.loadPlugins()
        rospy.loginfo(str(nb_plugins_found) + " plugins found")

        self.plugins = self.manager.getPluginsOfCategory(["Default"])
        self.plugins += self.manager.getPluginsOfCategory(["etherCAT_hand"])

        self.plugin_actions = []

        cyberglove_menu = QtGui.QMenu("Cyberglove")
        cyberglove_menu.setIcon(QtGui.QIcon(self.parent.rootPath + '/images/icons/iconGlove.png'))
        hand_menu = QtGui.QMenu("Shadow Hand")
        hand_menu.setIcon(QtGui.QIcon(self.parent.rootPath + '/images/icons/iconHand.png'))
        etherCAT_hand_menu = QtGui.QMenu("EtherCAT Hand")
        etherCAT_hand_menu.setIcon(QtGui.QIcon(self.parent.rootPath + '/images/icons/iconHand.png'))
        arm_menu = QtGui.QMenu("Shadow Arm")
        arm_menu.setIcon(QtGui.QIcon(self.parent.rootPath + '/images/icons/iconArm.png'))

        categories = [[cyberglove_menu, "cyberglove", 0],
                      [hand_menu, "shadowhand", 0],
                      [etherCAT_hand_menu, "etherCAT_hand", 0],
                      [arm_menu, "shadowarm", 0],
                      [QtGui.QMenu("Other"), "", 0]]

        for plugin in self.plugins:
            plugin.set_parent(self)
            name = plugin.plugin_name

            action = QtGui.QAction(name, self)
            action.setDisabled(True)
            self.plugin_actions.append([action, plugin])
            self.connect(action, QtCore.SIGNAL('triggered()'), plugin.activate)

            for category in categories:
                if category[1] in plugin.description.lower() or category[1] in plugin.category:
                    category[0].addAction(action)
                    category[2] = category[2] + 1
                    break

        for category in categories:
            if category[1] == "":
                tools.addSeparator()
            if category[2] == 0:
                category[0].setDisabled(True)
            tools.addMenu(category[0])
        self.parent.statusBar().showMessage(str(len(self.plugins)) + ' plugins loaded.', 500)

        ####
        # Enable / Disable plugins depending on the loaded libraries
        ##
        self.timer = QtCore.QTimer(self)
        self.connect(self.timer, QtCore.SIGNAL('timeout()'), self.refresh_activated_plugins)
        self.timer.setInterval(1000 / Config.main_widget_refresh_rate)
        self.timer.start()

        ####
        # Add view menu
        ##
        view = self.parent.menuBar().addMenu('&Window')
        action = QtGui.QAction("Tile the windows", self)
        self.connect(action, QtCore.SIGNAL('triggered()'), self.tile)
        view.addAction(action)
        action = QtGui.QAction("Cascade the windows", self)
        self.connect(action, QtCore.SIGNAL('triggered()'), self.cascade)
        view.addAction(action)

    def tile(self):
        self.container.tileSubWindows()

    def cascade(self):
        self.container.cascadeSubWindows()

    def refresh_activated_plugins(self):
        """
        refresh the activated plugins by checking if the ros nodes on which the
        plugin depends are started or not.
        """
        no_plugin_available = True
        for action_and_plugin in self.plugin_actions:
            dependencies = action_and_plugin[1].depends()
            activate_plugin = True
            if dependencies == None:
                activate_plugin = True
                no_plugin_available = False
            else:
                for dependency in dependencies:
                    if self.parent.libraries[dependency].status != "started":
                        activate_plugin = False

                    if activate_plugin:
                        no_plugin_available = False

            action_and_plugin[0].setEnabled(activate_plugin)

        if no_plugin_available:
            self.parent.statusBar().showMessage('No Plugins Available. Start the Libraries you need...', 2000)
            self.parent.robot_and_libraries_dock.show()

    def on_close(self):
        for plugin in self.plugins:
            try:
                plugin.on_close()
            except:
                pass



#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_control_gui')
import rospy
from shadowhand_ros import ShadowHand_ROS

import logging
#enables the logging used by yapsy
logging.basicConfig(level=logging.ERROR)

from yapsy.PluginManager import PluginManager, IPlugin
from PyQt4 import Qt, QtCore, QtGui, QtWebKit
import os, sys

class MainWidget(QtGui.QWidget):
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
        nb_plugins_found = self.manager.locatePlugins()
        rospy.loginfo(str(nb_plugins_found) + " plugins found")
        self.manager.loadPlugins()
                
        self.plugins = self.manager.getPluginsOfCategory("Default")
        
        self.plugin_actions = []
        plugin_id = 0
        for plugin in self.plugins:
            plugin.plugin_object.set_parent(self)
            name = plugin.plugin_object.name
            
            plugin.plugin_object.id = plugin_id
            action = QtGui.QAction(name, self)
            self.plugin_actions.append(action)
            self.connect(action, QtCore.SIGNAL('triggered()'), plugin.plugin_object.activate)
            tools.addAction(action)
            plugin_id += 1
                        
        self.parent.statusBar().showMessage(str(len(self.plugins)) + ' plugins loaded.', 500)

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
        
        
        
        
        
        
        

#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_control_gui')

import rospy

import logging
#enables the logging used by yapsy
logging.basicConfig(level=logging.ERROR)

import subprocess
from yapsy.PluginManager import PluginManager
from PyQt4 import QtCore, QtGui, Qt
import os, sys


from main_widget import MainWidget

class ReloadGraspSignalWidget(Qt.QWidget):
    reloadGraspSig = QtCore.pyqtSignal(int)
    
    def __init__(self, parent = None):
        super(ReloadGraspSignalWidget, self).__init__(parent)

class MainWindow(QtGui.QMainWindow):
    def __init__(self):
        QtGui.QMainWindow.__init__(self)
        
        ####
        # BASIC PARAMETERS
        ##        
        self.setWindowTitle("Shadow Robot Controller")
        self.resize(1000, 600)
        self.setWindowIcon(QtGui.QIcon('images/icons/app_icon.png'))
                
        ####
        # TOOLBAR
        ##
        self.exit = QtGui.QAction(QtGui.QIcon('images/icons/application-exit.png'), 'Exit', self)
        self.exit.setStatusTip('Exit application')
        self.connect(self.exit, QtCore.SIGNAL('triggered()'), QtCore.SLOT('close()'))
        self.toolbar = self.addToolBar('Exit')
        self.toolbar.addAction(self.exit)

        #Shortcuts
        self.exit.setShortcut('Ctrl+Q')

        ####
        # MENUBAR
        ##        
        self.menubar = self.menuBar()
        file = self.menubar.addMenu('&File')
        file.addAction(self.exit)
        
        tools = self.menubar.addMenu('&Tools')
        rxgraph = QtGui.QAction('RxPlot', self)
        self.connect(rxgraph, QtCore.SIGNAL('triggered()'), self.launch_rxgraph)
        robot_monitor = QtGui.QAction('Robot Monitor', self)
        self.connect(robot_monitor, QtCore.SIGNAL('triggered()'), self.launch_robot_monitor)
        
        tools.addAction(rxgraph)
        tools.addAction(robot_monitor)

        ###
        # SIGNALS
        ##
        self.reload_grasp_signal_widget = ReloadGraspSignalWidget()            

        ####
        # LIBRARIES
        ##
        self.libraries = {}

        ####
        # MAIN WIDGET
        ##
        self.setCentralWidget(MainWidget(self))

        ####
        # STATUSBAR
        ##
        self.statusBar().showMessage('Ready', 2000)

    def launch_rxgraph(self):
        subprocess.Popen("rxgraph".split())

    def launch_robot_monitor(self):
        subprocess.Popen("rosrun robot_monitor robot_monitor".split())


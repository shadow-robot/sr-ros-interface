#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_control_gui')

import rospy

import subprocess
import logging
#enables the logging used by yapsy
logging.basicConfig(level=logging.ERROR)

import subprocess
from yapsy.PluginManager import PluginManager
from PyQt4 import QtCore, QtGui, Qt
import os, sys

process = subprocess.Popen("rospack find sr_control_gui".split(), stdout=subprocess.PIPE)
rootPath = process.communicate()[0]
rootPath = rootPath.split('\n')
rootPath = rootPath[0]
sys.path.append(rootPath + '/src/sr_control_gui/dock_widgets')

from robot_and_libraries_dock_widget import RobotAndLibrariesDockWidget
from robot_and_libraries_backend import RobotAndLibrariesBackend
from main_widget import MainWidget

class ReloadGraspSignalWidget(Qt.QWidget):
    reloadGraspSig = QtCore.pyqtSignal(int)
    
    def __init__(self, parent=None):
        super(ReloadGraspSignalWidget, self).__init__(parent)


class MainWindow(QtGui.QMainWindow):
    def __init__(self):
        QtGui.QMainWindow.__init__(self)
        rospy.init_node('sr_control_gui')
        
        # root path
        process = subprocess.Popen("rospack find sr_control_gui".split(), stdout=subprocess.PIPE)
        self.rootPath = process.communicate()[0]
        self.rootPath = self.rootPath.split('\n')
        self.rootPath = self.rootPath[0]
        ####
        # BASIC PARAMETERS
        ##        
        self.setWindowTitle("Shadow Robot Controller")
        self.resize(1000, 600)
        self.setWindowIcon(QtGui.QIcon(self.rootPath + '/images/icons/app_icon.png'))
                
        ####
        # TOOLBAR
        ##
        self.exit = QtGui.QAction(QtGui.QIcon(self.rootPath + '/images/icons/application-exit.png'), 'Exit', self)
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
        rxgraph = QtGui.QAction('RxGraph', self)
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
        # DOCKS
        ##
        self.robot_and_libraries_backend = RobotAndLibrariesBackend()
        
        self.robot_and_libraries_dock = RobotAndLibrariesDockWidget(self, self.robot_and_libraries_backend)
        
        self.show_robot_and_libraries = QtGui.QAction('Show Robot / Ros nodes', self)
        self.show_robot_and_libraries.setStatusTip('Robot and libraries')
        self.connect(self.show_robot_and_libraries, QtCore.SIGNAL('triggered()'), self.robot_and_libraries_dock.show_hide)
        
        spacer = QtGui.QWidget()
        spacer.setSizePolicy(Qt.QSizePolicy.Expanding, Qt.QSizePolicy.Expanding)
                
        self.toolbar_docks = self.addToolBar('Docks')
        self.toolbar_docks.addWidget(spacer)
        self.toolbar_docks.addAction(self.show_robot_and_libraries)
        
        self.addDockWidget(QtCore.Qt.TopDockWidgetArea, self.robot_and_libraries_dock)

        ####
        # LIBRARIES
        ##
        self.libraries = self.robot_and_libraries_backend.libraries
                
        ####
        # MAIN WIDGET
        ##
        my_mdi_area = MainWidget(self)
        self.setCentralWidget(my_mdi_area)

        ####
        # STATUSBAR
        ##
        self.statusBar().showMessage('Ready', 2000)
        self.connect(my_mdi_area, QtCore.SIGNAL("messageToStatusbar(QString)"),
                     self.statusBar(), QtCore.SLOT("showMessage(QString)"))

    def launch_rxgraph(self):
        subprocess.Popen("rxgraph".split())

    def launch_robot_monitor(self):
        subprocess.Popen("rosrun robot_monitor robot_monitor".split())


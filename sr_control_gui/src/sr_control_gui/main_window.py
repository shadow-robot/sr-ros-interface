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
import subprocess
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
    """
    A Qt signal send when the grasps list has been changed. All the plugins
    using the grasps subscribes to this signal and reload their grasps lists.
    """
    reloadGraspSig = QtCore.pyqtSignal(int)

    def __init__(self, parent=None):
        super(ReloadGraspSignalWidget, self).__init__(parent)

class ReloadObjectSignalWidget(Qt.QWidget):
    """
    A Qt signal send when the grasps list has been changed. All the plugins
    using the grasps subscribes to this signal and reload their grasps lists.
    """
    reloadObjectSig = QtCore.pyqtSignal(int)

    def __init__(self, parent=None):
        super(ReloadObjectSignalWidget, self).__init__(parent)

class MainWindow(QtGui.QMainWindow):
    """
    The main window's main widget is MainWidget.
    """
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
        self.reload_object_signal_widget = ReloadObjectSignalWidget()

        ####
        # DOCKS
        ##
        self.show_robot_and_libraries = QtGui.QAction('Show Robot / Ros nodes', self)
        self.show_robot_and_libraries.setIcon(QtGui.QIcon(self.rootPath + '/images/icons/show.png'))
        self.show_robot_and_libraries.setStatusTip('Robot and libraries')

        self.robot_and_libraries_backend = RobotAndLibrariesBackend()
        self.robot_and_libraries_dock = RobotAndLibrariesDockWidget(self, self.robot_and_libraries_backend)

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
        self.my_mdi_area = MainWidget(self)
        self.setCentralWidget(self.my_mdi_area)

        ####
        # STATUSBAR
        ##
        self.statusBar().showMessage('Ready', 2000)
        self.connect(self.my_mdi_area, QtCore.SIGNAL("messageToStatusbar(QString)"),
                     self.statusBar(), QtCore.SLOT("showMessage(QString)"))

    def launch_rxgraph(self):
        subprocess.Popen("rxgraph".split())

    def launch_robot_monitor(self):
        subprocess.Popen("rosrun robot_monitor robot_monitor".split())

    def closeEvent(self, event):
        self.my_mdi_area.on_close()

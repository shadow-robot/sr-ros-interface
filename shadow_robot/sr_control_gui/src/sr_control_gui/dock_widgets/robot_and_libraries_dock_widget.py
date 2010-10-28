
import roslib; roslib.load_manifest('sr_control_gui')

import rospy

from PyQt4 import QtCore, QtGui, Qt

from generic_dock_widget import GenericDockWidget
from robot_and_libraries_backend import RobotAndLibrariesBackend, Library

library_refresh_rate = 0.5


class LibraryItem(QtGui.QTreeWidgetItem):
    def __init__(self, library, parent):
        data = []
        data.append(library.name)
        data.append(library.status)
        data.append(library.hostname)
        
        self.library = library
        
        QtGui.QTreeWidgetItem.__init__(self, data)
        
        self.menu = QtGui.QMenu(data[0])
        
        self.green = QtGui.QColor(153, 231, 96)
        self.red = QtGui.QColor(236, 178, 178)
        self.orange = QtGui.QColor(253, 166, 15)
        
        self.start_lib = QtGui.QAction('Start', self.menu)
        self.menu.connect(self.start_lib, QtCore.SIGNAL('triggered()'), self.start_library)
        
        self.stop_lib = QtGui.QAction('Stop', self.menu)
        self.menu.connect(self.stop_lib, QtCore.SIGNAL('triggered()'), self.stop_library)
        
        self.menu.addAction(self.start_lib)
        self.menu.addAction(self.stop_lib)
        
        timer = QtCore.QTimer(parent)
        parent.connect(timer, QtCore.SIGNAL('timeout()'), self.refresh_status)
        timer.start(1000 / library_refresh_rate)
    
    def refresh_status(self):
        self.library.get_status()
        current_status = self.library.status
        self.setData(1, QtCore.Qt.DisplayRole, current_status)
        if "ing" in current_status:
            self.start_lib.setDisabled(True)
            self.stop_lib.setDisabled(True)
            
            self.setBackgroundColor(0, QtGui.QColor(self.orange))
        if current_status == "started":
            self.start_lib.setDisabled(True)
            self.stop_lib.setEnabled(True)
            
            self.setBackgroundColor(0, QtGui.QColor(self.green))
        if current_status == "stopped":
            self.start_lib.setEnabled(True)
            self.stop_lib.setDisabled(True)
            
            self.setBackgroundColor(0, QtGui.QColor(self.red))
            
    
    def showContextMenu(self, point):
        self.menu.exec_(point)

    def stop_library(self):
        self.library.stop()
        self.setData(1, QtCore.Qt.DisplayRole, "stopping")
        self.setBackgroundColor(0, QtGui.QColor(self.orange))

    def start_library(self):
        self.library.start()
        self.setData(1, QtCore.Qt.DisplayRole, "starting")
        self.setBackgroundColor(0, QtGui.QColor(self.orange))

class LibrariesWidget(QtGui.QWidget):
    def __init__(self, parent):
        QtGui.QWidget.__init__(self, parent=parent)
               
        layout = QtGui.QHBoxLayout()

        list_frame = QtGui.QFrame(self)
        listframe_layout = QtGui.QVBoxLayout()
        
        self.robot_and_libraries_backend = RobotAndLibrariesBackend()
        
        list_of_nodes = ["/shadowhand"]
        
        self.robot_and_libraries_backend.add_library("Shadow Hand",
                                                     list_of_nodes=list_of_nodes,
                                                     start_cmd="roslaunch sr_hand srh_motor.launch",
                                                     stop_cmd="rosnode kill " + " ".join(list_of_nodes),
                                                     status_cmd="rosnode list")
        
        list_of_nodes = ["/shadowarm",
                          "/shadowhand",
                          "/srh_robot_state_publisher_pos",
                          "/srh_robot_state_publisher_target",
                          "/fixed_frame_pos_pub_arm",
                          "/fixed_frame_target_pub_arm",
                          "/link_hand_arm_pos",
                          "/link_hand_arm_target",
                          "/robot_state_publisher_pos_arm",
                          "/robot_state_publisher_target_arm"]
        
        self.robot_and_libraries_backend.add_library("Shadow Hand and Arm",
                                                     list_of_nodes=list_of_nodes,
                                                     start_cmd="roslaunch sr_hand sr_arm_motor.launch",
                                                     stop_cmd="rosnode kill " + " ".join(list_of_nodes),
                                                     status_cmd="rosnode list")
        
        list_of_nodes = ["/cyberglove"]
        self.robot_and_libraries_backend.add_library("Cyberglove",
                                                     list_of_nodes=list_of_nodes,
                                                     start_cmd="roslaunch cyberglove cyberglove.launch",
                                                     stop_cmd="rosnode kill " + " ".join(list_of_nodes),
                                                     status_cmd="rosnode list")
        
        list_of_nodes = ["/cyberglove_remapper"]
        self.robot_and_libraries_backend.add_library("Glove Remapper",
                                                     list_of_nodes=list_of_nodes,
                                                     start_cmd="roslaunch sr_remappers remapper_glove.launch",
                                                     stop_cmd="rosnode kill " + " ".join(list_of_nodes),
                                                     status_cmd="rosnode list")

        self.tree = QtGui.QTreeWidget()
        self.tree.setHeaderLabels(["ROS Library", "Status", "Computer"])
        items = []
        for lib in self.robot_and_libraries_backend.libraries.values():
            items.append(LibraryItem(lib, self.tree))
        
        self.tree.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.connect(self.tree,
                     QtCore.SIGNAL("customContextMenuRequested(const QPoint &)"),
                     self.showContextMenu) 
        
        for item in items:
            self.tree.addTopLevelItem(item)
            
        listframe_layout.addWidget(self.tree)
        list_frame.setLayout(listframe_layout)

        layout.addWidget(list_frame)
        self.setLayout(layout)
        
    def showContextMenu(self, point):
        item = self.tree.itemAt(point)
        point.setY(point.y() + 30)
        point = self.tree.mapToGlobal(point)
        item.showContextMenu(point)
        
        
class RobotsWidget(QtGui.QWidget):
    def __init__(self, parent):
        QtGui.QWidget.__init__(self, parent=parent)
        layout = QtGui.QVBoxLayout()
        frame = QtGui.QFrame(self)
        
        layout.addWidget(frame)
        self.setLayout(layout)

class RobotAndLibrariesDockWidget(GenericDockWidget):
    def __init__(self, parent):
        GenericDockWidget.__init__(self, parent=parent)
        
        
        frame = QtGui.QFrame()
        layout = QtGui.QHBoxLayout()
        
        libraries = LibrariesWidget(self)
        robots = RobotsWidget(self)
        
        layout.addWidget(libraries)
        layout.addWidget(robots)

        frame.setLayout(layout)
        self.setWidget(frame)
        
        
    

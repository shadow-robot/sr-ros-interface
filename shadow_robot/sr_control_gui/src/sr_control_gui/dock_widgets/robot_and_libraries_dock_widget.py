
import roslib; roslib.load_manifest('sr_control_gui')

import rospy

from PyQt4 import QtCore, QtGui, Qt

from generic_dock_widget import GenericDockWidget

class LibraryItem(QtGui.QTreeWidgetItem):
    def __init__(self, data):
        QtGui.QTreeWidgetItem.__init__(self, data)
        self.menu = QtGui.QMenu(data[0])
        
        self.is_active = False
        self.green = QtGui.QColor(153, 231, 96)
        self.red = QtGui.QColor(231, 153, 96)
        
        activate_lib = QtGui.QAction('Activate', self.menu)
        self.menu.connect(activate_lib, QtCore.SIGNAL('triggered()'), self.activate_library)
        
        disactivate_lib = QtGui.QAction('Disactivate', self.menu)
        self.menu.connect(disactivate_lib, QtCore.SIGNAL('triggered()'), self.disactivate_library)
        
        self.menu.addAction(activate_lib)
        self.menu.addAction(disactivate_lib)
        
    def showContextMenu(self, point):
        self.menu.exec_(point)

    def disactivate_library(self):
        print "disactivating library"
        self.is_active = False
        self.setData(1, QtCore.Qt.DisplayRole, "stopped")
        #self.setBackgroundColor(0, QtGui.QColor(self.red))

    def activate_library(self):
        print "activating library"
        self.is_active = True
        self.setData(1, QtCore.Qt.DisplayRole, "started")
        #self.setBackgroundColor(0, QtGui.QColor(self.green))

class LibrariesWidget(QtGui.QWidget):
    def __init__(self, parent):
        QtGui.QWidget.__init__(self, parent=parent)
        layout = QtGui.QHBoxLayout()

        list_frame = QtGui.QFrame(self)
        listframe_layout = QtGui.QVBoxLayout()
        

        self.tree = QtGui.QTreeWidget()
        self.tree.setHeaderLabels(["ROS Library", "Status"])
        items = []
        items.append(LibraryItem(["Shadow Hand", "stopped"]))
        items.append(LibraryItem(["Shadow Arm", "stopped"]))
        items.append(LibraryItem(["Cyberglove", "stopped"]))
        items.append(LibraryItem(["Remapper", "stopped"]))
        
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
        
        
    

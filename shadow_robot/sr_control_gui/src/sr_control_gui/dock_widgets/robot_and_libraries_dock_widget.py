
import roslib; roslib.load_manifest('sr_control_gui')

import rospy

from PyQt4 import QtCore, QtGui, Qt

from generic_dock_widget import GenericDockWidget
from robot_and_libraries_backend import RobotAndLibrariesBackend, Library

library_refresh_rate = 0.5


class LoginForm(QtGui.QDialog):
    def __init__(self, parent, treeitem, title, library):
        QtGui.QDialog.__init__(self, parent)
        self.library = library
        self.treeitem = treeitem
        
        self.setWindowTitle(title)
        self.setModal(True)

        dialog_frame = QtGui.QFrame(self)
        dialog_layout = QtGui.QGridLayout()
        
        label_login = QtGui.QLabel(dialog_frame)
        label_login.setText("Login: ")
        self.login_input = QtGui.QLineEdit(dialog_frame)    
        dialog_layout.addWidget(label_login, 0, 0)
        dialog_layout.addWidget(self.login_input, 0, 1)    
        
        dialog_frame.connect(self.login_input, QtCore.SIGNAL('textChanged(QString)'),
                             self.login_changed)
        
        label_pwd = QtGui.QLabel(dialog_frame)
        label_pwd.setText("Password: ")
        self.pwd_input = QtGui.QLineEdit(dialog_frame)
        self.pwd_input.setEchoMode(QtGui.QLineEdit.Password)
        dialog_layout.addWidget(label_pwd, 1, 0)
        dialog_layout.addWidget(self.pwd_input, 1, 1)
 
        dialog_frame.connect(self.pwd_input, QtCore.SIGNAL('textChanged(QString)'),
                             self.pwd_changed)
        
        dialog_frame.setLayout(dialog_layout)
        
        btn_frame = QtGui.QFrame()   
        self.btn_ok = QtGui.QPushButton(btn_frame)
        self.btn_ok.setText("OK")
        self.btn_ok.setDisabled(True)
        self.connect(self.btn_ok, QtCore.SIGNAL("clicked()"), self.accept)
        btn_cancel = QtGui.QPushButton(btn_frame)
        btn_cancel.setText("Cancel")
        self.connect(btn_cancel, QtCore.SIGNAL("clicked()"), self.reject)
        
        btn_layout = QtGui.QHBoxLayout()
        btn_layout.addWidget(self.btn_ok)
        btn_layout.addWidget(btn_cancel)
        btn_frame.setLayout(btn_layout)
        
        layout = QtGui.QVBoxLayout()
        layout.addWidget(dialog_frame)
        layout.addWidget(btn_frame)
        self.setLayout(layout)

        self.show()
    
    def login_changed(self, login):
        if login != "":
            if str(self.pwd_input.text()) != "":
                self.btn_ok.setEnabled(True)
                return
        self.btn_ok.setDisabled(True)
        
    def pwd_changed(self, pwd):
        if pwd != "":
            if str(self.login_input.text()) != "":
                self.btn_ok.setEnabled(True)
                return
        self.btn_ok.setDisabled(True)
    
    def accept(self):
        login = str(self.login_input.text())
        pwd = str(self.pwd_input.text())
        
        good_pwd = self.library.test_login_pwd(login, pwd)
        
        if good_pwd:
            self.library.login = login
            self.library.password = pwd
            self.treeitem.reboot_computer_action.setEnabled(True)
            self.treeitem.setIcon(0, QtGui.QIcon(self.library.icon_remote_path))
        else:
            rospy.logerr("Wrong password - setting local IP")
            self.reject()
        
        #restarting the timer
        self.treeitem.timer.start(1000 / library_refresh_rate)
        QtGui.QDialog.accept(self)
        
    def reject(self):
        self.library.set_local_ip()
        self.treeitem.setData(3, QtCore.Qt.DisplayRole, self.library.hostname)
        self.treeitem.reboot_computer_action.setDisabled(True)
        self.treeitem.setIcon(0, QtGui.QIcon(self.library.icon_local_path))
        
        QtGui.QDialog.reject(self)

class LibraryItem(QtGui.QTreeWidgetItem):
    def __init__(self, library, parent):
        data = []
        data.append("")
        data.append(library.name)
        data.append(library.status)
        data.append(library.hostname)
        self.library = library
        self.parent = parent
        QtGui.QTreeWidgetItem.__init__(self, data)
        
        self.setIcon(1, QtGui.QIcon(self.library.icon_library_path))
        self.menu = QtGui.QMenu(data[0])
        
        self.green = QtGui.QColor(153, 231, 96)
        self.red = QtGui.QColor(236, 178, 178)
        self.orange = QtGui.QColor(253, 166, 15)
        
        self.start_lib = QtGui.QAction('Start', self.menu)
        self.menu.connect(self.start_lib, QtCore.SIGNAL('triggered()'), self.start_library)
        
        self.stop_lib = QtGui.QAction('Stop', self.menu)
        self.menu.connect(self.stop_lib, QtCore.SIGNAL('triggered()'), self.stop_library)
        
        self.reboot_computer_action = QtGui.QAction('Reboot the computer', self.menu)
        self.menu.connect(self.reboot_computer_action, QtCore.SIGNAL('triggered()'), self.reboot_computer)
        
        self.edit_ip_action = QtGui.QAction('Modify the IP address', self.menu)
        self.menu.connect(self.edit_ip_action, QtCore.SIGNAL('triggered()'), self.edit_ip)
        
        
        if self.library.is_local:
            self.reboot_computer_action.setDisabled(True)
            self.setIcon(0, QtGui.QIcon(self.library.icon_local_path))
        
        self.menu.addAction(self.start_lib)
        self.menu.addAction(self.stop_lib)
        self.menu.addSeparator()
        self.menu.addAction(self.reboot_computer_action)
        self.menu.addSeparator()
        self.menu.addAction(self.edit_ip_action)
       
        self.timer = QtCore.QTimer(parent)
        parent.connect(self.timer, QtCore.SIGNAL('timeout()'), self.refresh_status)
        self.timer.start(1000 / library_refresh_rate)
            
    def edit_ip(self):
        self.setFlags(self.flags() | QtCore.Qt.ItemIsEditable)
        self.parent.editItem(self, 3)
    
    def refresh_status(self):
        self.library.get_status()
        current_status = self.library.status
        self.setData(2, QtCore.Qt.DisplayRole, current_status)
        if "ing" in current_status:
            self.start_lib.setDisabled(True)
            self.stop_lib.setDisabled(True)
            
            self.setBackgroundColor(2, QtGui.QColor(self.orange))
        if current_status == "started":
            self.start_lib.setDisabled(True)
            self.stop_lib.setEnabled(True)
            
            self.setBackgroundColor(2, QtGui.QColor(self.green))
        if current_status == "stopped":
            self.start_lib.setEnabled(True)
            self.stop_lib.setDisabled(True)
            
            self.setBackgroundColor(2, QtGui.QColor(self.red))
            
    def setData(self, column, role, value):
        """
        overload the virtual setData method to read the new ip
        """
        QtGui.QTreeWidgetItem.setData(self, column, role, value)
        # if the ip address is changed
        if column == 3:
            try:
                value = str(value.toString())
                self.change_ip(value)
            except:
                return - 1
    
    def showContextMenu(self, point):
        self.menu.exec_(point)

    def stop_library(self):
        self.library.stop()
        self.setData(2, QtCore.Qt.DisplayRole, "stopping")
        self.setBackgroundColor(0, QtGui.QColor(self.orange))

    def start_library(self):
        self.library.start()
        self.setData(2, QtCore.Qt.DisplayRole, "starting")
        self.setBackgroundColor(0, QtGui.QColor(self.orange))

    def reboot_computer(self):
        print "rebooting"
    
    def change_ip(self, new_ip):
        self.timer.stop()
        if self.library.set_ip(new_ip) == -1:
            rospy.logerr("Bad IP address specified - setting local IP")
            self.library.set_local_ip()
        self.setData(3, QtCore.Qt.DisplayRole, self.library.hostname)
        if self.library.is_local:
            self.reboot_computer_action.setDisabled(True)
            self.setIcon(0, QtGui.QIcon(self.library.icon_local_path))
        else:
            LoginForm(self.parent, self, self.library.hostname + " login", self.library)
        
    
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
                                                     status_cmd="rosnode list",
                                                     root_path=self.parent().parent().rootPath)
        
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
                                                     status_cmd="rosnode list",
                                                     root_path=self.parent().parent().rootPath)
        
        list_of_nodes = ["/cyberglove"]
        self.robot_and_libraries_backend.add_library("Cyberglove",
                                                     list_of_nodes=list_of_nodes,
                                                     start_cmd="roslaunch cyberglove cyberglove.launch",
                                                     stop_cmd="rosnode kill " + " ".join(list_of_nodes),
                                                     status_cmd="rosnode list",
                                                     root_path=self.parent().parent().rootPath)
        
        list_of_nodes = ["/cyberglove_remapper"]
        self.robot_and_libraries_backend.add_library("Glove Remapper",
                                                     list_of_nodes=list_of_nodes,
                                                     start_cmd="roslaunch sr_remappers remapper_glove.launch",
                                                     stop_cmd="rosnode kill " + " ".join(list_of_nodes),
                                                     status_cmd="rosnode list",
                                                     root_path=self.parent().parent().rootPath)
        
        self.robot_and_libraries_backend.add_robot("Robot Hand",
                                                   root_path=self.parent().parent().rootPath)

        self.tree = QtGui.QTreeWidget()
        self.tree.setEditTriggers(Qt.QAbstractItemView.DoubleClicked)
        
        self.tree.setHeaderLabels(["", "ROS Nodes / Robot Code", "Status", "Computer"])
        items = []
        
        for lib in self.robot_and_libraries_backend.libraries.values():
            items.append(LibraryItem(lib, self.tree))
        
        self.tree.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.connect(self.tree,
                     QtCore.SIGNAL("customContextMenuRequested(const QPoint &)"),
                     self.showContextMenu) 
                
        self.connect(self.tree, QtCore.SIGNAL('itemDoubleClicked (QTreeWidgetItem *, int)'),
                     self.edit_ip)
        for item in items:
            self.tree.addTopLevelItem(item)
        listframe_layout.addWidget(self.tree)
        
        self.tree.resizeColumnToContents(0)
        self.tree.resizeColumnToContents(1)
        #don't resize col 2 and 3 as they contain dynamic content
        
        list_frame.setLayout(listframe_layout)

        layout.addWidget(list_frame)
        self.setLayout(layout)
    
    def edit_ip(self, item, value):
        item.edit_ip()
    
    def showContextMenu(self, point):
        item = self.tree.itemAt(point)
        if item == None:
            return
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
        
        
    

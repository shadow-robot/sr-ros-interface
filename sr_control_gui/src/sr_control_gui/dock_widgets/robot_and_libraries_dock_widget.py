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

from generic_dock_widget import GenericDockWidget
from robot_and_libraries_backend import RobotAndLibrariesBackend, Library
from config import *

class LoginForm(QtGui.QDialog):
    """
    A login form when trying to connect through ssh to a distant computer.
    """
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
        cursor = Qt.QCursor()
        cursor.setShape(QtCore.Qt.WaitCursor)
        self.setCursor(cursor)

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
            cursor.setShape(QtCore.Qt.ArrowCursor)
            self.setCursor(cursor)

            self.reject()

        cursor.setShape(QtCore.Qt.ArrowCursor)
        self.setCursor(cursor)
        #restarting the timer
        self.treeitem.timer.start()
        QtGui.QDialog.accept(self)

    def reject(self):
        self.library.set_local_ip()
        self.treeitem.setData(3, QtCore.Qt.DisplayRole, self.library.hostname)
        self.treeitem.reboot_computer_action.setDisabled(True)
        self.treeitem.setIcon(0, QtGui.QIcon(self.library.icon_local_path))

        QtGui.QDialog.reject(self)

class LibraryItem(QtGui.QTreeWidgetItem):
    """
    A QTreeWidgetItem used to display / edit the information for a library (ROS node) item.
    """
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
        self.bright_red = QtGui.QColor(255, 78, 78)
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

        self.error_starting_stopping_status = ""
        self.error_starting_stopping_timer = QtCore.QTimer(parent)
        parent.connect(self.error_starting_stopping_timer, QtCore.SIGNAL('timeout()'),
                       self.check_error_starting_stopping)
        self.error_starting_stopping_timer.setSingleShot(True)
        self.error_starting_stopping_timer.setInterval(Config.library_timeout * 1000)

        self.timer = QtCore.QTimer(parent)
        parent.connect(self.timer, QtCore.SIGNAL('timeout()'), self.refresh_status)
        self.timer.setInterval(1000 / Config.library_refresh_rate)
        self.timer.start()

    def edit_ip(self):
        self.setFlags(self.flags() | QtCore.Qt.ItemIsEditable)
        self.parent.editItem(self, 3)

    def check_error_starting_stopping(self):
        print "Timeout"
        self.library.status = "Error"
        self.setData(2, QtCore.Qt.DisplayRole, "Error")
        self.setBackgroundColor(2, QtGui.QColor(self.bright_red))
        self.start_lib.setEnabled(True)
        self.stop_lib.setEnabled(True)

    def refresh_status(self):
        self.library.get_status()
        current_status = self.library.status
        self.setData(2, QtCore.Qt.DisplayRole, current_status)

        if current_status == "starting" or current_status == "stopping":
            if not self.error_starting_stopping_timer.isActive():
                self.error_starting_stopping_status = current_status

                self.error_starting_stopping_timer.start()
            self.start_lib.setDisabled(True)
            self.stop_lib.setDisabled(True)
            self.setBackgroundColor(2, QtGui.QColor(self.orange))

        if current_status == "started":
            if self.error_starting_stopping_timer.isActive():
                if self.error_starting_stopping_status == "starting":
                    self.error_starting_stopping_timer.stop()
            self.start_lib.setDisabled(True)
            self.stop_lib.setEnabled(True)

            self.setBackgroundColor(2, QtGui.QColor(self.green))
        if current_status == "stopped":
            if self.error_starting_stopping_timer.isActive():
                if self.error_starting_stopping_status == "stopping":
                    self.error_starting_stopping_timer.stop()
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
        self.setBackgroundColor(2, QtGui.QColor(self.orange))

    def start_library(self):
        self.library.start()
        self.setData(2, QtCore.Qt.DisplayRole, "starting")
        self.setBackgroundColor(2, QtGui.QColor(self.orange))

    def reboot_computer(self):
        rospy.logerr("reboot not yet implemented")

    def change_ip(self, new_ip):
        cursor = Qt.QCursor()
        cursor.setShape(QtCore.Qt.WaitCursor)
        self.parent.setCursor(cursor)

        self.timer.stop()
        if new_ip == "local":
            self.library.set_local_ip()
        elif self.library.set_ip(new_ip) == -1:
            rospy.logerr("Bad IP address specified - setting local IP")
            self.library.set_local_ip()
        self.setData(3, QtCore.Qt.DisplayRole, self.library.hostname)
        if self.library.is_local:
            self.reboot_computer_action.setDisabled(True)
            self.setIcon(0, QtGui.QIcon(self.library.icon_local_path))
        else:
            LoginForm(self.parent, self, self.library.hostname + " login", self.library)

        cursor.setShape(QtCore.Qt.ArrowCursor)
        self.parent.setCursor(cursor)

    def close(self):
        self.timer.stop()
        try:
            self.library.ssh_client.close()
        except:
            #the ssh client was already closed. Do nothing
            nothing = True

class LibrariesWidget(QtGui.QWidget):
    """
    Add the correct library to the libraries dict. And contains the QTreeWidget which will display
    the info regarding the different available libraries.
    """
    def __init__(self, parent, backend):
        QtGui.QWidget.__init__(self, parent=parent)

        layout = QtGui.QHBoxLayout()

        list_frame = QtGui.QFrame(self)
        listframe_layout = QtGui.QVBoxLayout()

        self.robot_and_libraries_backend = backend

        self.robot_and_libraries_backend.add_library(Config.library_shadowhand.name,
                                                     list_of_nodes=Config.library_shadowhand.list_of_nodes,
                                                     start_cmd=Config.library_shadowhand.start_cmd,
                                                     stop_cmd=Config.library_shadowhand.stop_cmd,
                                                     status_cmd=Config.library_shadowhand.status_cmd,
                                                     root_path=self.parent().parent().rootPath)

        self.robot_and_libraries_backend.add_library(Config.library_shadowarm.name,
                                                     list_of_nodes=Config.library_shadowarm.list_of_nodes,
                                                     start_cmd=Config.library_shadowarm.start_cmd,
                                                     stop_cmd=Config.library_shadowarm.stop_cmd,
                                                     status_cmd=Config.library_shadowarm.status_cmd,
                                                     root_path=self.parent().parent().rootPath)

	self.robot_and_libraries_backend.add_library(Config.library_shadow_arm_hand.name,
                                                     list_of_nodes=Config.library_shadow_arm_hand.list_of_nodes,
                                                     start_cmd=Config.library_shadow_arm_hand.start_cmd,
                                                     stop_cmd=Config.library_shadow_arm_hand.stop_cmd,
                                                     status_cmd=Config.library_shadow_arm_hand.status_cmd,
                                                     root_path=self.parent().parent().rootPath)

        self.robot_and_libraries_backend.add_library(Config.library_cyberglove.name,
                                                     list_of_nodes=Config.library_cyberglove.list_of_nodes,
                                                     start_cmd=Config.library_cyberglove.start_cmd,
                                                     stop_cmd=Config.library_cyberglove.stop_cmd,
                                                     status_cmd=Config.library_cyberglove.status_cmd,
                                                     root_path=self.parent().parent().rootPath)

        self.robot_and_libraries_backend.add_library(Config.library_cyberglove_remapper.name,
                                                     list_of_nodes=Config.library_cyberglove_remapper.list_of_nodes,
                                                     start_cmd=Config.library_cyberglove_remapper.start_cmd,
                                                     stop_cmd=Config.library_cyberglove_remapper.stop_cmd,
                                                     status_cmd=Config.library_cyberglove_remapper.status_cmd,
                                                     root_path=self.parent().parent().rootPath)

        self.robot_and_libraries_backend.add_robot(Config.robot_code.name,
                                                   root_path=self.parent().parent().rootPath)

        self.tree = QtGui.QTreeWidget()
        self.tree.setEditTriggers(Qt.QAbstractItemView.DoubleClicked)

        self.tree.setHeaderLabels(["", "ROS Nodes / Robot Code", "Status", "Computer", "Info"])

        self.items = []

        for lib in self.robot_and_libraries_backend.libraries.values():
            self.items.append(LibraryItem(lib, self.tree))

        self.tree.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.connect(self.tree,
                     QtCore.SIGNAL("customContextMenuRequested(const QPoint &)"),
                     self.showContextMenu)

        self.connect(self.tree, QtCore.SIGNAL('itemDoubleClicked (QTreeWidgetItem *, int)'),
                     self.edit_ip)
        for item in self.items:
            self.tree.addTopLevelItem(item)
        listframe_layout.addWidget(self.tree)

        self.tree.resizeColumnToContents(0)
        self.tree.resizeColumnToContents(1)
        self.tree.resizeColumnToContents(4)
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

    def close(self):
        for item in self.items:
            item.close()

        QtGui.QWidget.close(self)

class RobotsWidget(QtGui.QWidget):
    def __init__(self, parent):
        QtGui.QWidget.__init__(self, parent=parent)
        layout = QtGui.QVBoxLayout()
        frame = QtGui.QFrame(self)

        layout.addWidget(frame)
        self.setLayout(layout)

class RobotAndLibrariesDockWidget(GenericDockWidget):
    """
    The dock containing the QTreeWidget which displays the robot / libraries info.
    """
    def __init__(self, parent, backend):
        GenericDockWidget.__init__(self, parent=parent)

        frame = QtGui.QFrame()
        layout = QtGui.QHBoxLayout()

        libraries = LibrariesWidget(self, backend)
        robots = RobotsWidget(self)

        layout.addWidget(libraries)
        layout.addWidget(robots)

        frame.setLayout(layout)
        self.setWidget(frame)




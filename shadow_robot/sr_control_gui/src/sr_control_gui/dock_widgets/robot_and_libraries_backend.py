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

from PyQt4 import QtCore, QtGui, Qt
import rospy

import sys
import paramiko
import subprocess
import socket
import time
from config import *

class RunCommand(Qt.QThread):
    """
    Run either a local or ssh command in a separate thread.
    """
    def __init__(self, command, final_status, library):
        Qt.QThread.__init__(self)
        self.command = command
        self.ip = library.ip
        self.library = library
        self.final_status = final_status

    def run(self):
        if self.command != "":
            if self.library.is_local:
                self.run_local_command()
            elif self.final_status == "get_status":
                #the get status command is a ros command run locally
                self.run_local_command()
            else:
                self.run_ssh_command()

    def run_local_command(self):
        """
        Run a local command using subprocess
        """
        process = subprocess.Popen(self.command.split(),
                                   stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)
        output = process.communicate()[0]

        #get status commands need special treatment of their output
        if self.final_status == "get_status":
            output = output.split('\n')

            all_name_found = True
            for name in self.library.list_of_nodes:
                name_found = False
                for line in output:
                    if name in output:
                        name_found = True
                        #remove the line from the output
                        output.pop(output.index(name))
                        break
                if not name_found:
                    all_name_found = False
                    break
            if all_name_found:
                if "stopping" != self.library.status:
                    self.library.status = "started"
            else:
                if "starting" != self.library.status:
                    self.library.status = "stopped"

        # get status command for the robot code needs a special treatment as well
        elif self.final_status == "get_status_robot":
            output = output.split('\n')
            if len(output) > 1:
                if "stopping" != self.library.status:
                    self.library.status = "started"
            else:
                if "starting" != self.library.status:
                    self.library.status = "stopped"

        else:
            self.library.status = self.final_status

    def run_ssh_command(self):
        """
        Run a command on a distant computer using ssh (paramiko library).
        """
        try:
            self.library.ssh_client.connect(self.library.ip,
                                            username=self.library.login,
                                            password=self.library.password)
        except:
            print "Error", sys.exc_info()[0]
            return

        stdin, stdout, stderr = self.library.ssh_client.exec_command("source ~/.bashrc.d/ros.sh; " +
                                                                     self.command)

        if self.final_status == "get_status_robot":
            output = stdout.readlines()
            if int(output[0].strip('\n')) > 0:
                if "stopping" != self.library.status:
                    self.library.status = "started"
            else:
                if "starting" != self.library.status:
                    self.library.status = "stopped"

        else:
            if stderr.readlines() != []:
                print stderr
                error = "Couldn't run the command \""
                error += self.command + "\"on the following ip: "
                error += self.library.ip
                for line in stderr.readlines():
                    error += "\n   " + line
                rospy.logerr(error)

        self.library.ssh_client.close()

class Library(object):
    """
    An object containing all the necessary information for a ROS node / library
    """
    def __init__(self, name="", list_of_nodes=[], start_cmd="", stop_cmd="", status_cmd="", root_path=""):
        self.name = name
        self.ip = ""
        self.hostname = ""
        self.status = ""
        self.login = ""
        self.password = ""
        self.icon_local_path = root_path + '/images/icons/local.png'
        self.icon_remote_path = root_path + '/images/icons/remote.png'
        self.icon_library_path = root_path + '/images/icons/ros.jpg'
        self.is_local = True
        self.ssh_client = paramiko.SSHClient()
        self.ssh_client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.list_of_nodes = list_of_nodes
        self.start_cmd = start_cmd
        self.stop_cmd = stop_cmd
        self.status_cmd = status_cmd
        self.thread_start = RunCommand(self.start_cmd, "started", self)
        self.thread_status = RunCommand(self.status_cmd, "get_status", self)
        self.thread_stop = RunCommand(self.stop_cmd, "stopped", self)
        self.set_local_ip()

    def set_local_ip(self):
        ip = socket.gethostbyname(socket.gethostname())
        self.set_ip(ip)
        self.is_local = True

    def set_ip(self, ip, login="", password=""):
        try:
            socket.inet_aton(ip)
        except socket.error:
            # not a valid ip
            return - 1
        self.ip = ip
        try:
            self.hostname = socket.gethostbyaddr(str(ip))[0]
        except:
            self.hostname = str(ip)

        if self.hostname != socket.gethostbyname(socket.gethostname()):
            self.login = login
            self.password = password
            self.is_local = False
        else:
            self.is_local = True

    def get_status(self):
        if not self.thread_status.isRunning():
            self.thread_status.start()

    def start(self):
        self.thread_start.wait(1000)
        self.status = "starting"
        self.thread_start.start()

    def stop(self):
        self.thread_stop.wait(1000)
        self.status = "stopping"
        self.thread_stop.start()

    def test_login_pwd(self, login, pwd):
        try:
            self.ssh_client.connect(self.ip, username=login, password=pwd)
        except:
            print "Error: ", sys.exc_info()[0]
            return False
        self.ssh_client.close()
        return True

class Robot(Library):
    """
    Contains all the needed information for the Robot code. Inherits from Library object
    because the two are almost the same.
    """
    def __init__(self, name="", root_path=""):
        start_cmd = Config.robot_code.start_cmd
        stop_cmd = Config.robot_code.stop_cmd
        #to check the status we check that /proc/robot/bus/ is not empty
        status_cmd = Config.robot_code.status_cmd
        Library.__init__(self, name=name, list_of_nodes=[],
                         start_cmd=start_cmd, stop_cmd=stop_cmd,
                         status_cmd=status_cmd, root_path=root_path)
        self.icon_library_path = root_path + '/images/icons/iconArmHand.png'

        self.thread_status = RunCommand(self.status_cmd, "get_status_robot", self)

class RobotAndLibrariesBackend(object):
    """
    Contains a dict with the different Libraries (ROS nodes)/ Robot interfaces.
    """
    def __init__(self):
        self.libraries = {}

    def add_library(self, name, list_of_nodes=[], start_cmd="", stop_cmd="", status_cmd="", root_path=""):
        lib = Library(name=name, list_of_nodes=list_of_nodes, start_cmd=start_cmd,
                      stop_cmd=stop_cmd, status_cmd=status_cmd, root_path=root_path)
        self.libraries[name] = lib

    def add_robot(self, name, root_path=""):
        rob = Robot(name=name, root_path=root_path)
        self.libraries[name] = rob



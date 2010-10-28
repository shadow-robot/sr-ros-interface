
import roslib; roslib.load_manifest('sr_control_gui')

from PyQt4 import QtCore, QtGui, Qt
import rospy

import subprocess
import socket
import time

class RunCommand(Qt.QThread):
    def __init__(self, command, final_status, library):
        Qt.QThread.__init__(self)
        self.command = command
        self.ip = library.ip
        self.library = library
        self.final_status = final_status
    
    def run(self):
        #TODO run the command
        if self.command != "":
            process = subprocess.Popen(self.command.split(), stdout=subprocess.PIPE)
            output = process.communicate()[0]
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
                    self.library.status = "started"
                else:
                    if "starting" != self.library.status:
                        self.library.status = "stopped"
                
            else:
                self.library.status = self.final_status
        
class Library(object):
    def __init__(self, name="", list_of_nodes=[], start_cmd="", stop_cmd="", status_cmd=""):
        self.name = name
        self.ip = ""
        self.hostname = ""
        self.status = ""
        self.is_local = True
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
    
    def set_ip(self, ip):
        self.ip = ip
        try:
            self.hostname = socket.gethostbyaddr(str(ip))[0]
            if self.hostname != socket.gethostbyname(socket.gethostname()):
                self.is_local = False
            else:
                self.is_local = True
        except:
            self.hostname = ""

    def get_status(self):
        if not self.thread_status.isRunning():
            self.thread_status.start()

    def start(self):
        self.status = "starting"
        self.thread_start.start()

    def stop(self):
        self.status = "stopping"
        self.thread_stop.start()

class RobotAndLibrariesBackend(object):
    def __init__(self):
        self.libraries = {}
        
    def add_library(self, name, list_of_nodes=[], start_cmd="", stop_cmd="", status_cmd=""):
        lib = Library(name=name, list_of_nodes=list_of_nodes, start_cmd=start_cmd,
                      stop_cmd=stop_cmd, status_cmd=status_cmd)
        self.libraries[name] = lib
        
    

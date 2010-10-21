import os, sys

#Not very pretty....
import subprocess
process = subprocess.Popen("rospack find sr_control_gui".split(), stdout=subprocess.PIPE)
rootPath = process.communicate()[0]
rootPath = rootPath.split('\n')
rootPath = rootPath[0]
sys.path.append(rootPath+ "/src/sr_control_gui/plugins")
  
from PyQt4 import QtCore, QtGui, Qt
from cyberglove_generic_plugin import CybergloveGenericPlugin

from cyberglove_calibrer import *
from cyberglove_mapper import *

class GloveMappingWidget(QtGui.QWidget):
    def __init__(self, parent, joint_names):
        QtGui.QWidget.__init__(self, parent=parent)
        self.frame = QtGui.QFrame()
        self.layout = QtGui.QGridLayout()
        self.layout.setHorizontalSpacing(10)
        self.layout.setVerticalSpacing(10)
        
        green = QtGui.QColor(153, 231, 96)
        red = QtGui.QColor(207, 103, 103)
        self.saved_palette = self.palette()
        self.green_palette = self.palette()
        self.green_palette.setBrush(Qt.QPalette.Window, green)
        
        self.red_palette = self.palette()
        self.red_palette.setBrush(Qt.QPalette.Window, red)
        
        col = 0
        #vectors to set the correct row in the layout for each col
        rows = [0, 0, 0, 0, 0, 0]
        
        self.joints_frames = {}
        
        for joint in joint_names:
            if "index" in joint.lower():
                col = 0    
            elif "middle" in joint.lower():
                col = 1
            elif "ring" in joint.lower():
                if "pinkie" in joint.lower():
                    col = 3
                else:
                    col = 2
            elif "pinkie" in joint.lower():
                col = 3
            elif "thumb" in joint.lower():
                col = 4
            else:
                col = 5
                
            row = rows[col]
            rows[col] = row + 1
            
            subframe = QtGui.QFrame()
            layout = QtGui.QHBoxLayout()
            name = QtGui.QLabel()
            name.setText(joint)
            layout.addWidget(name)
            subframe.setLayout(layout)
            subframe.setPalette(self.red_palette)
            subframe.setAutoFillBackground(True)  
            subframe.repaint()
            self.joints_frames[joint] = subframe
            self.layout.addWidget(subframe, row, col)
        
        self.set_calibrated(joint_names)
        
        self.frame.setLayout(self.layout)
        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.frame)
        self.frame.show()
        self.setLayout(layout)
        self.show()
        
    def set_not_calibrated(self, joints):
        for joint in joints:
            self.joints_frames[joint].setPalette(self.red_palette)
            self.frame.repaint()      
            
    def set_calibrated(self, joints):
        for joint in joints:
            self.joints_frames[joint].setPalette(self.green_palette)
            self.frame.repaint()      

class Cyberglove(CybergloveGenericPlugin):  
    name = "Cyberglove"
        
    def __init__(self):
        CybergloveGenericPlugin.__init__(self)

        self.frame = QtGui.QFrame()
        self.layout = QtGui.QHBoxLayout()
        
        self.frame.setLayout(self.layout)
        self.window.setWidget(self.frame)
        
        self.is_activated = False

    def activate(self):
        CybergloveGenericPlugin.activate(self)
        self.set_icon(self.parent.parent.rootPath + '/src/sr_control_gui/images/icons/iconGlove.png')
        if self.is_activated:
            return
        
        self.is_activated = True

        self.calibrer = CybergloveCalibrer()
        
        joint_names = self.parent.parent.libraries["cyberglove"].joints.keys()
        joint_names.sort()
        self.glove_mapping_widget = GloveMappingWidget(self.frame, joint_names)
        self.layout.addWidget(self.glove_mapping_widget)
        
        Qt.QTimer.singleShot(0, self.window.adjustSize)
        
        
        
        
        
        
        
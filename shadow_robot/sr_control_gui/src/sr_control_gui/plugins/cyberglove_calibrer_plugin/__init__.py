import os, sys

#Not very pretty....
import subprocess
process = subprocess.Popen("rospack find sr_control_gui".split(), stdout=subprocess.PIPE)
rootPath = process.communicate()[0]
rootPath = rootPath.split('\n')
rootPath = rootPath[0]
sys.path.append(rootPath + "/src/sr_control_gui/plugins")
  
from PyQt4 import QtCore, QtGui, Qt
from cyberglove_generic_plugin import CybergloveGenericPlugin

from cyberglove_calibrer import *
from cyberglove_mapper import *

import subprocess
process = subprocess.Popen("rospack find sr_control_gui".split(), stdout=subprocess.PIPE)
rootPath = process.communicate()[0]
rootPath = rootPath.split('\n')
rootPath = rootPath[0]
noimage_path = rootPath + '/src/sr_control_gui/images/image-missing.png'

class StepDescription():
    def __init__(self):
        self.text = ""
        self.image_path = noimage_path

class StepDescriber(QtGui.QWidget):
    def __init__(self, parent):
        QtGui.QWidget.__init__(self, parent=parent)
        
        self.description = StepDescription
        self.frame = QtGui.QFrame()
        self.layout = QtGui.QVBoxLayout()
        
        self.text_description = QtGui.QTextEdit()
        self.layout.addWidget(self.text_description)    
        
        self.image_description = QtGui.QLabel()
        self.layout.addWidget(self.image_description)  
        
        self.frame.setLayout(self.layout)
        layout = QtGui.QHBoxLayout() 
        layout.addWidget(self.frame)
        self.setLayout(layout)
        self.show()
    
    def set_description(self, description):
        self.text_description.setText(description.text)
        self.image_description.setPixmap(QtGui.QPixmap(description.image_path))
    
    
        
class StepSelector(QtGui.QWidget):
    def __init__(self, parent, calibrer):
        QtGui.QWidget.__init__(self, parent=parent)
        self.frame = QtGui.QFrame()
        self.calibrer = calibrer
        self.layout = QtGui.QVBoxLayout()
        self.layout.setSpacing(5)
        self.steps = {}
        
        self.title = QtGui.QLabel()
        self.title.setText("Calibration Steps")
        
        self.current_substep = 0
        
        self.step_describer = StepDescriber(self)
        
        self.list = QtGui.QListWidget()
        first_item = self.refresh_list()  
        self.connect(self.list, QtCore.SIGNAL('itemClicked(QListWidgetItem*)'), self.step_choosed)
        self.list.setViewMode(QtGui.QListView.ListMode)
        self.list.setResizeMode(QtGui.QListView.Adjust)
        self.list.setItemSelected(first_item, True)
        self.step_choosed(first_item, first_time=True)
        self.layout.addWidget(self.title)
        self.layout.addWidget(self.list)
        
        self.frame.setLayout(self.layout)
        layout = QtGui.QHBoxLayout()
        
        layout.addWidget(self.frame)
        layout.addWidget(self.step_describer)
        self.setLayout(layout)
        self.show()
        
    def step_choosed(self, item, first_time=False):
        step_name = str(item.text())
        
        description = StepDescription()
        description.text = self.steps[step_name].step_description[self.current_substep]
        self.step_describer.set_description(description)
        
    
    def refresh_list(self, value = 0):
        self.list.clear()   
        first_item = None
        steps = self.calibrer.calibration_steps
        steps.sort()
        for step in steps:
            item = QtGui.QListWidgetItem(step.step_name)
            if first_item == None:
                first_item = item
            self.list.addItem(item)
            self.steps[step.step_name] = step
        return first_item
        
class GloveMappingWidget(QtGui.QWidget):
    def __init__(self, parent, joint_names):
        QtGui.QWidget.__init__(self, parent=parent)
        self.frame = QtGui.QFrame()
        self.layout = QtGui.QGridLayout()
        self.layout.setHorizontalSpacing(5)
        self.layout.setVerticalSpacing(5)
        
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

    def calibrate_current_step(self):
        all_joints_calibrated = False
        
        print "calibrating"
        
        return all_joints_calibrated
              
    def save_calib(self, path):
        print "saving"
    
    def load_calib(self, path):
        print "loading"


class CybergloveCalibrerPlugin(CybergloveGenericPlugin):  
    name = "Cyberglove Calibrer"
        
    def __init__(self):
        CybergloveGenericPlugin.__init__(self)

        self.frame = QtGui.QFrame()
        self.layout = QtGui.QVBoxLayout()
        
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
        
        subframe = QtGui.QFrame()
        sublayout = QtGui.QHBoxLayout()
        
        self.step_selector = StepSelector(self.frame, self.calibrer)
        sublayout.addWidget(self.step_selector)
        
        btn_frame = QtGui.QFrame()
        btn_layout = QtGui.QVBoxLayout()
        btn_layout.setSpacing(25)
        btn_calibrate = QtGui.QPushButton()
        btn_calibrate.setText("Calibrate")
        btn_calibrate.setToolTip("Calibrate the current selected step")
        btn_calibrate.setIcon(QtGui.QIcon(self.parent.parent.rootPath + '/src/sr_control_gui/images/icons/calibrate.png'))
        btn_layout.addWidget(btn_calibrate)
        btn_frame.connect(btn_calibrate, QtCore.SIGNAL('clicked()'), self.calibrate_current_step)
        
        self.btn_save = QtGui.QPushButton()
        self.btn_save.setText("Save")
        self.btn_save.setToolTip("Save the current calibration")
        self.btn_save.setIcon(QtGui.QIcon(self.parent.parent.rootPath + '/src/sr_control_gui/images/icons/save.png'))
        self.btn_save.setDisabled(True)
        btn_layout.addWidget(self.btn_save)
        btn_frame.connect(self.btn_save, QtCore.SIGNAL('clicked()'), self.save_calib)
        
        btn_load = QtGui.QPushButton()
        btn_load.setText("Load")
        btn_load.setToolTip("Load a Glove calibration")
        btn_load.setIcon(QtGui.QIcon(self.parent.parent.rootPath + '/src/sr_control_gui/images/icons/load.png'))
        btn_layout.addWidget(btn_load)
        btn_frame.connect(btn_load, QtCore.SIGNAL('clicked()'), self.load_calib)
        
        btn_frame.setLayout(btn_layout)
        sublayout.addWidget(btn_frame)
        
        subframe.setLayout(sublayout)
        self.layout.addWidget(subframe)
        
        Qt.QTimer.singleShot(0, self.window.adjustSize)
        
    def calibrate_current_step(self):
        if all_joints_calibrated:
            self.btn_save.setEnabled(True)
        
    def save_calib(self):
        filename = QtGui.QFileDialog.getSaveFileName(self.window, 'Save Calibration', '')
        if filename == "":
            return
        
        self.calibrer.save_calib(filename)
    
    def load_calib(self):
        filename = QtGui.QFileDialog.getOpenFileName(self.window, 'Open Calibration', '')
        if filename == "":
            return
        
        self.calibrer.load_calib(filename)
        
        
        
        
        

#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_control_gui')
import rospy

from yapsy.IPlugin import IPlugin

from PyQt4 import QtCore, QtGui, Qt

from shadow_generic_plugin import ShadowGenericPlugin

import math

class Joint():
    def __init__(self, name="", min=0, max=90):
        self.name = name
        self.min = min
        self.max = max

class ExtendedSlider(QtGui.QWidget):
    def __init__(self, joint, plugin_parent):
        QtGui.QWidget.__init__(self)
        self.plugin_parent = plugin_parent
        self.name = joint.name 
        
        self.slider = QtGui.QSlider()
        self.slider.setOrientation(QtCore.Qt.Vertical)
        self.slider.setFocusPolicy(QtCore.Qt.NoFocus)
        self.slider.setMinimum(joint.min)
        self.slider.setMaximum(joint.max)
        self.slider.setInvertedControls(True)
        self.slider.setInvertedAppearance(True)
        self.slider.setTracking(False)
                         
        self.label = QtGui.QLabel(self) 
        self.label.setText(joint.name)         
        self.position = QtGui.QLabel(self) 
        self.position.setText("Pos: " + str(0))        
        self.target = QtGui.QLabel(self) 
        self.target.setText("Target: " + str(0)) 

        self.layout = QtGui.QGridLayout()#QVBoxLayout()
        self.layout.setAlignment(QtCore.Qt.AlignCenter)
        self.layout.setMargin(1) 
        self.layout.setSpacing(2)
        self.layout.addWidget(self.label, 0, 0) 
        self.layout.addWidget(self.slider, 1, 0)
        self.layout.addWidget(self.position, 2, 0) 
        self.layout.addWidget(self.target, 3, 0) 
        self.layout.setColumnMinimumWidth(0, 60)
        
        self.is_selected = False
        self.current_value = 0
        self.selected = QtGui.QCheckBox('', self)
        self.selected.setFocusPolicy(QtCore.Qt.NoFocus)
        self.connect(self.selected, QtCore.SIGNAL('stateChanged(int)'), self.checkbox_click)
        self.layout.addWidget(self.selected)
        self.connect(self.slider, QtCore.SIGNAL('valueChanged(int)'), self.changeValue)
        
        self.timer = Qt.QTimer(self)
        self.connect(self.timer, QtCore.SIGNAL('timeout()'), self.update)
        
        self.setLayout(self.layout) 
        self.show()
        
    def changeValue(self, value):
        self.target.setText("Target: " + str(value))
        self.sendupdate(value)
    
    def sendupdate(self, value):
        joint_dict = {}
        joint_dict[self.name] = value
        self.plugin_parent.sendupdate(joint_dict)        
    
    def update(self):
        self.current_value = round(self.plugin_parent.parent.parent.libraries["sr_library"].valueof(self.name),1)
        self.position.setText("Pos: " + str(self.current_value))
    
    def checkbox_click(self, value):
        self.is_selected = value

class ExtendedSuperSlider(ExtendedSlider):
    def __init__(self, joint, plugin_parent):
        ExtendedSlider.__init__(self, joint, plugin_parent)
        self.plugin_parent = plugin_parent
        self.position.close()
        self.target.setText("Target: 0%")
        self.layout.removeWidget(self.selected)
        self.selected.close()

    def changeValue(self, value):
        #modify the values from the selected sliders.
        joint_dict = {}
        for slider in self.plugin_parent.sliders:
            if slider.is_selected:
                temp_value = (slider.slider.maximum() - slider.slider.minimum()) * float(value) / 100.0
                slider.slider.setSliderPosition(temp_value)
                joint_dict[slider.name] = temp_value
        
        self.current_value = value
        self.plugin_parent.sendupdate(joint_dict)
        self.target.setText("Target: " + str(value) + "%")
    
    def update(self):
        return

class JointSlider(ShadowGenericPlugin):  
    name = "Joint Slider"
    
    def __init__(self, joints_list):
        ShadowGenericPlugin.__init__(self)
        
        self.layout = QtGui.QHBoxLayout()
        self.layout.setAlignment(QtCore.Qt.AlignCenter)
        self.frame = QtGui.QFrame()

        #Add the sliders
        self.sliders = []
        for joint in joints_list:
            slider = ExtendedSlider(joint, self)
            self.layout.addWidget(slider)
            self.sliders.append(slider)

        #Add a slider to control all the selected sliders
        selected_joints = Joint("Move Selected Joints", -100, 100)
        self.super_slider = ExtendedSuperSlider(selected_joints, self)
        self.layout.addWidget(self.super_slider)
            
        self.frame.setLayout(self.layout)
        self.window.setWidget(self.frame)
        
    def sendupdate(self, dict):
        rospy.logerr("Virtual method, please implement.")
    
    def activate(self):
        ShadowGenericPlugin.activate(self)
        for slider in self.sliders:
            slider.timer.start(200)
        
            

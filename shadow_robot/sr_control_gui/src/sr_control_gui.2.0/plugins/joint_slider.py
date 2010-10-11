from yapsy.IPlugin import IPlugin

from PyQt4 import QtCore, QtGui, Qt

import os, sys
sys.path.append(os.getcwd() + "/plugins")
from generic_plugin import GenericPlugin

class Joint():
    def __init__(self, name="", min=0, max=90):
        self.name = name
        self.min = min
        self.max = max

class ExtendedSlider(QtGui.QWidget):
    def __init__(self, parent, joint):
        QtGui.QWidget.__init__(self)
        self.slider = QtGui.QSlider()
        self.slider.setOrientation(QtCore.Qt.Vertical)
        self.slider.setFocusPolicy(QtCore.Qt.NoFocus)
        self.slider.setMinimum(joint.min)
        self.slider.setMaximum(joint.max)
         
        self.label = QtGui.QLabel(self) 
        self.label.setText(joint.name)         
        self.position = QtGui.QLabel(self) 
        self.position.setText("Pos: "+ str(0))        
        self.target = QtGui.QLabel(self) 
        self.target.setText("Target: "+str(0)) 

        layout = QtGui.QVBoxLayout()
        layout.setAlignment(QtCore.Qt.AlignCenter)
        layout.setMargin(1) 
        layout.setSpacing(2) 
        layout.addWidget(self.label) 
        layout.addWidget(self.slider)
        layout.addWidget(self.position) 
        layout.addWidget(self.target) 
        
        self.is_selected = False
        self.current_value = 0
        self.selected = QtGui.QCheckBox('',self)
        self.selected.setFocusPolicy(QtCore.Qt.NoFocus)
        self.connect(self.selected, QtCore.SIGNAL('stateChanged(int)'), self.checkbox_click)
        layout.addWidget(self.selected)
        
        self.setLayout(layout) 
        self.connect(self.slider, QtCore.SIGNAL('valueChanged(int)'), self.changeValue)
        self.show()
        
    def changeValue(self, value):
        self.current_value = value
        self.target.setText("Target: " + str(value))
    
    def checkbox_click(self, value):
        self.is_selected = value

class ExtendedSuperSlider(ExtendedSlider):
    def __init__(self, parent, joint, slider_parent):
        ExtendedSlider.__init__(self, parent, joint)
        self.slider_parent = slider_parent

    def changeValue(self, value):
        #read the values from the selected sliders.
        for slider in self.slider_parent.sliders:
            if slider.is_selected:
                print slider.current_value
        

class JointSlider(GenericPlugin):  
    name = "Joint Slider"
    
    def __init__(self, joints_map):
        GenericPlugin.__init__(self)
        
        self.layout = QtGui.QHBoxLayout()
        self.layout.setAlignment(QtCore.Qt.AlignCenter)
        self.frame = QtGui.QFrame()

        #Add the sliders
        self.sliders = []
        for joint in joints_map.values():
            slider = ExtendedSlider(self.frame, joint)
            self.layout.addWidget(slider)
            self.sliders.append(slider)

        #Add a slider to control all the selected sliders
        selected_joints = Joint("Move Selected Joints", -100, 100)
        self.super_slider = ExtendedSuperSlider(self.frame, selected_joints, self)
        self.layout.addWidget(self.super_slider)
            
        self.frame.setLayout(self.layout)
        self.window.setWidget(self.frame)
            

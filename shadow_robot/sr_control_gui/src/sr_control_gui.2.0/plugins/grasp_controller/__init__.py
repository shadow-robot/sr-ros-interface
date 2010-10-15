import os, sys
sys.path.append(os.getcwd() + "/plugins")

from PyQt4 import QtCore, QtGui, Qt
from shadow_generic_plugin import ShadowGenericPlugin

from Grasp import Grasp
from grasps_interpoler import GraspInterpoler
from grasps_parser import GraspParser

class GraspChooser(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)

        sublayout = QtGui.QVBoxLayout()
        frame = QtGui.QFrame()
        list = QtGui.QListWidget()
        #tree.setColumnCount(1)
        item = QtGui.QListWidgetItem("Toto", list)
        
        sublayout.addWidget(list)
        
        frame.setLayout(sublayout)
        self.show()
        
class GraspSlider(QtGui.QWidget):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        

class GraspController(ShadowGenericPlugin):  
    name = "Grasp Controller"
        
    def __init__(self):        
        ShadowGenericPlugin.__init__(self)
        self.set_icon('images/icons/iconHand.png')
        
        self.frame = QtGui.QFrame()
        self.layout = QtGui.QHBoxLayout()
        
        subframe = QtGui.QFrame()
        sublayout = QtGui.QVBoxLayout()
         
        self.grasp_slider = GraspSlider()
        sublayout.addWidget(self.grasp_slider)
        
        btn_frame = QtGui.QFrame()
        btn_layout = QtGui.QHBoxLayout()
        btn_save = QtGui.QPushButton()
        btn_save.setText("Save")
        btn_save.setFixedWidth(130)
        btn_frame.connect(btn_save, QtCore.SIGNAL('clicked()'), self.save_grasp)
        btn_layout.addWidget(btn_save)
        btn_set_ref = QtGui.QPushButton()
        btn_set_ref.setText("Set As Reference")
        btn_set_ref.setFixedWidth(130)
        btn_frame.connect(btn_set_ref, QtCore.SIGNAL('clicked()'), self.set_reference_grasp)
        btn_layout.addWidget(btn_set_ref)
        
        btn_frame.setLayout(btn_layout)
        sublayout.addWidget(btn_frame)
        subframe.setLayout(sublayout)
         
        self.layout.addWidget(subframe)
        
        self.grasp_chooser = GraspChooser()
        self.layout.addWidget(self.grasp_chooser)
             
        self.frame.show()  
        self.frame.setLayout(self.layout)
        self.window.setWidget(self.frame)
        
    def activate(self):
        ShadowGenericPlugin.activate(self)
        
        
    def save_grasp(self):
        print "save grasp"
    
    def set_reference_grasp(self):
        print "set ref grasp"

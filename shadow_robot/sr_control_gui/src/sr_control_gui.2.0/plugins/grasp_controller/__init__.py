import os, sys
sys.path.append(os.getcwd() + "/plugins")

from PyQt4 import QtCore, QtGui, Qt
from shadow_generic_plugin import ShadowGenericPlugin

from Grasp import Grasp
from grasps_interpoler import GraspInterpoler
from grasps_parser import GraspParser

class GraspChooser(QtGui.QListWidget):
    def __init__(self, parent=None):
        QtGui.QListWidget.__init__(self)

        self.parent = parent
        
        self.setViewMode(QtGui.QListView.ListMode)
        self.setResizeMode(QtGui.QListView.Adjust)

        #sublayout = QtGui.QVBoxLayout()
        #self.frame = QtGui.QFrame()
        #self.list = QtGui.QListWidget()
        #tree.setColumnCount(1)
    
    def draw(self):
        for grasp_name in self.parent.sr_library.grasp_parser.grasps.keys():
            self.addItem( QtGui.QListWidgetItem(grasp_name))    
        self.connect(self, QtCore.SIGNAL('itemClicked(QListWidgetItem*)'), self.grasp_choosed)
        
        #sublayout.addWidget(self.list)
        
        #self.frame.setLayout(sublayout)
        #self.show()
        
    def grasp_choosed(self, item):
        print item.text()
        #print grasp_name
        
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
        
        self.grasp_chooser = GraspChooser(self)
        self.layout.addWidget(self.grasp_chooser)
             
    def activate(self):
        ShadowGenericPlugin.activate(self)
        self.frame.show()  
        self.frame.setLayout(self.layout)
        self.window.setWidget(self.frame)
        self.grasp_chooser.draw()
        Qt.QTimer.singleShot(0, self.window.adjustSize)
        
        
    def save_grasp(self):
        print "save grasp"
    
    def set_reference_grasp(self):
        print "set ref grasp"

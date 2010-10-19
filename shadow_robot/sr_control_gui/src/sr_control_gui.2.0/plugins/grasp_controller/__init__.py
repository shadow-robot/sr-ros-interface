import os, sys
sys.path.append(os.getcwd() + "/plugins")

from PyQt4 import QtCore, QtGui, Qt
from shadow_generic_plugin import ShadowGenericPlugin

from Grasp import Grasp
from grasps_interpoler import GraspInterpoler
from grasps_parser import GraspParser

class GraspChooser(QtGui.QWidget):
    def __init__(self, parent, plugin_parent, title):
        QtGui.QWidget.__init__(self)
        self.plugin_parent = plugin_parent
        self.grasp = None
        self.title = QtGui.QLabel()
        self.title.setText(title)
    
    def draw(self):
        self.frame = QtGui.QFrame(self)

        self.list = QtGui.QListWidget()
        first_item = None
        for grasp_name in self.plugin_parent.sr_library.grasp_parser.grasps.keys():
            item = QtGui.QListWidgetItem(grasp_name)
            if first_item == None:
                first_item = item
            self.list.addItem(item)    
        self.connect(self.list, QtCore.SIGNAL('itemClicked(QListWidgetItem*)'), self.grasp_choosed)
        self.list.setViewMode(QtGui.QListView.ListMode)
        self.list.setResizeMode(QtGui.QListView.Adjust)
        self.list.setItemSelected(first_item, True)
        self.grasp_choosed(first_item, first_time = True)
        
        self.layout = QtGui.QVBoxLayout()
        self.layout.addWidget(self.title)
        self.layout.addWidget(self.list)
        
        self.frame.setLayout(self.layout)
        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.frame)
        self.frame.show()
        self.setLayout(layout)
        self.show()
        
    def grasp_choosed(self, item, first_time = False):
        self.grasp = self.plugin_parent.sr_library.grasp_parser.grasps[str(item.text())]
        if not first_time:
            self.plugin_parent.grasp_changed()
        #print grasp_name
        
class GraspSlider(QtGui.QWidget):
    def __init__(self, parent, plugin_parent):
        QtGui.QWidget.__init__(self, parent)
        self.plugin_parent = plugin_parent
        
    def draw(self):
        self.frame = QtGui.QFrame(self)
        label_frame = QtGui.QFrame(self.frame)
        from_label = QtGui.QLabel()
        from_label.setText("From")
        ref_label = QtGui.QLabel()
        ref_label.setText("Reference")
        to_label = QtGui.QLabel()
        to_label.setText("To")
        label_layout = QtGui.QHBoxLayout()
        label_layout.addWidget(from_label)
        label_layout.addWidget(ref_label)
        label_layout.addWidget(to_label)
        
        label_frame.setLayout(label_layout)
        
        self.slider = QtGui.QSlider()
        self.slider.setOrientation(QtCore.Qt.Horizontal)
        self.slider.setFocusPolicy(QtCore.Qt.NoFocus)
        self.slider.setTickInterval(100)
        self.slider.setTickPosition(Qt.QSlider.TicksAbove)
        self.slider.setMinimum(-100)
        self.slider.setMaximum(100)
        
        self.connect(self.slider, QtCore.SIGNAL('valueChanged(int)'), self.changeValue)
        
        self.layout = QtGui.QVBoxLayout()
        self.layout.addWidget(label_frame)
        self.layout.addWidget(self.slider)
                
        self.frame.setLayout(self.layout)
        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.frame)
        self.frame.show()
        self.setLayout(layout)
        self.show()
        
    def changeValue(self, value):
        self.plugin_parent.interpolate_grasps(value)

class GraspController(ShadowGenericPlugin):  
    name = "Grasp Controller"
        
    def __init__(self):        
        ShadowGenericPlugin.__init__(self)
        
        self.current_grasp = Grasp()
        self.current_grasp.name = 'CURRENT_UNSAVED'
        self.grasp_interpoler_1 = None
        self.grasp_interpoler_2 = None
        
        self.set_icon('images/icons/iconHand.png')
        
        self.frame = QtGui.QFrame()
        self.layout = QtGui.QHBoxLayout()
        
        subframe = QtGui.QFrame()
        sublayout = QtGui.QVBoxLayout()
         
        self.grasp_slider = GraspSlider(self.frame, self)
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
         
        self.grasp_from_chooser = GraspChooser(self.window, self, "From: ")
        self.layout.addWidget(self.grasp_from_chooser)
        self.layout.addWidget(subframe)
        
        self.grasp_to_chooser = GraspChooser(self.window, self, "To: ")
        self.layout.addWidget(self.grasp_to_chooser)
             
    def activate(self):
        ShadowGenericPlugin.activate(self)

        self.frame.setLayout(self.layout)
        self.window.setWidget(self.frame)
        self.grasp_slider.draw()
        self.grasp_to_chooser.draw()
        self.grasp_from_chooser.draw()
        self.set_reference_grasp()
        Qt.QTimer.singleShot(0, self.window.adjustSize)
        
    def save_grasp(self):
        print "save grasp"
    
    def set_reference_grasp(self):
        self.current_grasp.joints_and_positions = self.sr_library.read_all_current_positions()
        
        self.grasp_interpoler_1 = GraspInterpoler(self.grasp_from_chooser.grasp, self.current_grasp)
        self.grasp_interpoler_2 = GraspInterpoler(self.current_grasp, self.grasp_to_chooser.grasp)
        
        self.grasp_slider.slider.setValue(0)
    
    def grasp_changed(self):
        self.current_grasp.joints_and_positions = self.sr_library.read_all_current_positions()
        self.grasp_interpoler_1 = GraspInterpoler(self.grasp_from_chooser.grasp, self.current_grasp)
        self.grasp_interpoler_2 = GraspInterpoler(self.current_grasp, self.grasp_to_chooser.grasp)    
    
    def interpolate_grasps(self, value):
        #from -> current
        if value < 0:
            targets_to_send = self.grasp_interpoler_1.interpolate(100 + value)
            self.sr_library.sendupdate_from_dict(targets_to_send)
        else:   #current -> to
            targets_to_send = self.grasp_interpoler_2.interpolate(value)
            self.sr_library.sendupdate_from_dict(targets_to_send)

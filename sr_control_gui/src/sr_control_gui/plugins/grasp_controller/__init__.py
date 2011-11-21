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
from shadow_generic_plugin import ShadowGenericPlugin

from Grasp import Grasp
from grasps_interpoler import GraspInterpoler
from grasps_parser import GraspParser

from main_window import ReloadGraspSignalWidget

class JointSelecter(QtGui.QWidget):
    """
    Select which joints to save in a new grasp
    """
    def __init__(self, parent, all_joints):
        QtGui.QWidget.__init__(self, parent=parent)
        self.frame = QtGui.QFrame()
        self.layout = QtGui.QGridLayout()
        self.checkboxes = []
        
        col = 0
        #vectors to set the correct row in the layout for each col
        rows = [0, 0, 0, 0, 0, 0]
        joint_names = all_joints.keys()
        joint_names.sort()
        for joint in joint_names:
            if "fj1" in joint.lower():
                continue
            if "fj2" in joint.lower():
                continue    
            if "ff" in joint.lower():
                col = 0
            elif "mf" in joint.lower():
                col = 1
            elif "rf" in joint.lower():
                col = 2
            elif "lf" in joint.lower():
                col = 3
            elif "th" in joint.lower():
                col = 4
            else:
                col = 5
                
            row = rows[col]
            rows[col] = row + 1
            cb = QtGui.QCheckBox(QtCore.QString(joint), self.frame)
            self.checkboxes.append(cb)
            self.layout.addWidget(cb, row, col)
                
        self.frame.setLayout(self.layout)
        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.frame)
        self.frame.show()
        self.setLayout(layout)
        self.show()
    
    def get_selected(self):
        joints = []
        for cb in self.checkboxes:
            if cb.isChecked():
                joints.append(str(cb.text()))
        
        return joints

    def select_all(self):
        for cb in self.checkboxes:
            cb.setChecked(True)
            
    def deselect_all(self):
        for cb in self.checkboxes:
            cb.setChecked(False)    

class GraspSaver(QtGui.QDialog):
    """
    Save a new grasp from the current joints positions.
    """
    def __init__(self, parent, all_joints, plugin_parent):
        QtGui.QDialog.__init__(self, parent)
        self.plugin_parent = plugin_parent
        self.all_joints = all_joints
        self.setModal(True)
        self.setWindowTitle("Save Grasp")
        
        self.grasp_name = ""
        
        self.upper_frame = QtGui.QFrame()
        self.upper_layout = QtGui.QHBoxLayout()
        label_name = QtGui.QLabel()
        label_name.setText("Grasp Name: ")
        name_widget = QtGui.QLineEdit()
        self.upper_frame.connect(name_widget, QtCore.SIGNAL('textChanged(QString)'), self.name_changed)
        
        self.upper_layout.addWidget(label_name)
        self.upper_layout.addWidget(name_widget)
        self.upper_frame.setLayout(self.upper_layout)
        
        select_all_frame = QtGui.QFrame()
        select_all_layout = QtGui.QHBoxLayout()
        btn_select_all = QtGui.QPushButton(select_all_frame)
        btn_select_all.setText("Select All")
        select_all_layout.addWidget(btn_select_all)
        self.connect(btn_select_all, QtCore.SIGNAL("clicked()"), self.select_all)
        btn_deselect_all = QtGui.QPushButton(select_all_frame)
        btn_deselect_all.setText("Deselect All")
        select_all_layout.addWidget(btn_deselect_all)
        self.connect(btn_deselect_all, QtCore.SIGNAL("clicked()"), self.deselect_all)
        select_all_frame.setLayout(select_all_layout)
                
        self.joint_selecter = JointSelecter(self, self.all_joints)
        
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
        
        self.layout = QtGui.QVBoxLayout()
        self.layout.addWidget(self.upper_frame)
        self.layout.addWidget(select_all_frame)
        self.layout.addWidget(self.joint_selecter)
        self.layout.addWidget(btn_frame)
        
        self.setLayout(self.layout)
        self.show()
        
    def select_all(self):
        self.joint_selecter.select_all()
        
    def deselect_all(self):
        self.joint_selecter.deselect_all()
    
    def name_changed(self, name):
        self.grasp_name = name
        if self.grasp_name != "":
            self.btn_ok.setEnabled(True)
        else:
            self.btn_ok.setDisabled(True)
        
    
    def accept(self):
        grasp = Grasp()
        grasp.grasp_name = self.grasp_name
                
        joints_to_save = self.joint_selecter.get_selected()
        if len(joints_to_save) == 0:
            joints_to_save = self.all_joints.keys()
        for joint_to_save in joints_to_save:
            grasp.joints_and_positions[joint_to_save] = self.all_joints[joint_to_save]
        
        self.plugin_parent.parent.parent.libraries["sr_library"].grasp_parser.write_grasp_to_file(grasp)
        
        self.plugin_parent.parent.parent.reload_grasp_signal_widget.reloadGraspSig['int'].emit(1)
        #self.plugin_parent.refresh_lists()
        
        QtGui.QDialog.accept(self)

class GraspChooser(QtGui.QWidget):
    """
    Choose a grasp from a list of grasps.
    """
    def __init__(self, parent, plugin_parent, title):
        QtGui.QWidget.__init__(self)
        self.plugin_parent = plugin_parent
        self.grasp = None
        self.title = QtGui.QLabel()
        self.title.setText(title)
    
    def draw(self):
        self.frame = QtGui.QFrame(self)

        self.list = QtGui.QListWidget()
        first_item = self.refresh_list()  
        self.connect(self.list, QtCore.SIGNAL('itemClicked(QListWidgetItem*)'), self.grasp_choosed)
            
        self.connect(self.list, QtCore.SIGNAL('itemDoubleClicked(QListWidgetItem*)'), self.double_click)
        self.list.setViewMode(QtGui.QListView.ListMode)
        self.list.setResizeMode(QtGui.QListView.Adjust)
        self.list.setItemSelected(first_item, True)
        self.grasp_choosed(first_item, first_time=True)
        
        self.layout = QtGui.QVBoxLayout()
        self.layout.addWidget(self.title)
        self.layout.addWidget(self.list)
        
        
        ###
        # SIGNALS
        ##
        self.plugin_parent.parent.parent.reload_grasp_signal_widget.reloadGraspSig['int'].connect(self.refresh_list)
        
        self.frame.setLayout(self.layout)
        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.frame)
        self.frame.show()
        self.setLayout(layout)
        self.show()
        
    def double_click(self, item):
        self.grasp = self.plugin_parent.parent.parent.libraries["sr_library"].grasp_parser.grasps[str(item.text())]
        self.plugin_parent.parent.parent.libraries["sr_library"].sendupdate_from_dict(self.grasp.joints_and_positions)
        self.plugin_parent.set_reference_grasp()
    
    def grasp_choosed(self, item, first_time=False):
        self.grasp = self.plugin_parent.parent.parent.libraries["sr_library"].grasp_parser.grasps[str(item.text())]
        if not first_time:
            self.plugin_parent.grasp_changed()
            self.plugin_parent.set_reference_grasp()
    
    def refresh_list(self, value = 0):
        self.list.clear()   
        first_item = None
        grasps = self.plugin_parent.parent.parent.libraries["sr_library"].grasp_parser.grasps.keys()
        grasps.sort()
        for grasp_name in grasps:
            item = QtGui.QListWidgetItem(grasp_name)
            if first_item == None:
                first_item = item
            self.list.addItem(item)
        return first_item
    
        
class GraspSlider(QtGui.QWidget):
    """
    Slide from one grasp to another.
    """
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
    """
    The grasp controller plugin: slide from one grasp to another, save new grasps
    """
    name = "Grasp Controller"
        
    def __init__(self):        
        ShadowGenericPlugin.__init__(self)
        
        self.current_grasp = Grasp()
        self.current_grasp.name = 'CURRENT_UNSAVED'
        self.grasp_interpoler_1 = None
        self.grasp_interpoler_2 = None
        
        self.frame = QtGui.QFrame()
        self.layout = QtGui.QHBoxLayout()
        
        subframe = QtGui.QFrame()
        sublayout = QtGui.QVBoxLayout()
         
        self.grasp_slider = GraspSlider(self.frame, self)
        sublayout.addWidget(self.grasp_slider)
        
        btn_frame = QtGui.QFrame()
        btn_layout = QtGui.QHBoxLayout()
        self.btn_save = QtGui.QPushButton()
        self.btn_save.setText("Save")
        self.btn_save.setFixedWidth(130)
        btn_frame.connect(self.btn_save, QtCore.SIGNAL('clicked()'), self.save_grasp)
        btn_layout.addWidget(self.btn_save)
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
        
        self.frame.setLayout(self.layout)
        self.window.setWidget(self.frame)
        
        self.is_activated = False
             
    def activate(self):
        ShadowGenericPlugin.activate(self)
        
        self.btn_save.setIcon(QtGui.QIcon(self.parent.parent.rootPath + '/images/icons/save.png'))
        self.set_icon(self.parent.parent.rootPath + '/images/icons/iconHand.png')
        
        if self.is_activated:
            return
        
        self.is_activated = True

        self.grasp_slider.draw()
        self.grasp_to_chooser.draw()
        self.grasp_from_chooser.draw()
        self.set_reference_grasp()
        Qt.QTimer.singleShot(0, self.window.adjustSize)
        
    def save_grasp(self):
        all_joints = self.parent.parent.libraries["sr_library"].read_all_current_positions()
        GraspSaver(self.window, all_joints, self)
    
    def set_reference_grasp(self):
        self.current_grasp.joints_and_positions = self.parent.parent.libraries["sr_library"].read_all_current_positions()
        
        self.grasp_interpoler_1 = GraspInterpoler(self.grasp_from_chooser.grasp, self.current_grasp)
        self.grasp_interpoler_2 = GraspInterpoler(self.current_grasp, self.grasp_to_chooser.grasp)
        
        self.grasp_slider.slider.setValue(0)
    
    def grasp_changed(self):
        self.current_grasp.joints_and_positions = self.parent.parent.libraries["sr_library"].read_all_current_positions()
        self.grasp_interpoler_1 = GraspInterpoler(self.grasp_from_chooser.grasp, self.current_grasp)
        self.grasp_interpoler_2 = GraspInterpoler(self.current_grasp, self.grasp_to_chooser.grasp)    
    
    def interpolate_grasps(self, value):
        #from -> current
        if value < 0:
            targets_to_send = self.grasp_interpoler_1.interpolate(100 + value)
            self.parent.parent.libraries["sr_library"].sendupdate_from_dict(targets_to_send)
        else:   #current -> to
            targets_to_send = self.grasp_interpoler_2.interpolate(value)
            self.parent.parent.libraries["sr_library"].sendupdate_from_dict(targets_to_send)

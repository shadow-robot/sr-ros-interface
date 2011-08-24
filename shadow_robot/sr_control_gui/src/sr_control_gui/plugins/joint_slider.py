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
from generic_plugin import GenericPlugin

import math

class Joint():
    """
    Contains the name, the min and the max for the joints.
    """
    def __init__(self, name="", min=0, max=90):
        self.name = name
        self.min = min
        self.max = max

class ExtendedSlider(QtGui.QWidget):
    """
    This slider displays the current position and the target as well.
    """
    def __init__(self, joint, plugin_parent, parent=None):
        QtGui.QWidget.__init__(self, parent)
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
        self.position.setText("P: " + str(0))
        self.target = QtGui.QLabel(self)
        self.target.setText("T: " + str(0))

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
        self.target.setText("T: " + str(value))
        self.sendupdate(value)

    def sendupdate(self, value):
        joint_dict = {}
        joint_dict[self.name] = value
        self.plugin_parent.sendupdate(joint_dict)

    def update(self):
        try:
            self.current_value = round(self.plugin_parent.parent.parent.libraries["sr_library"].valueof(self.name),1)
            self.position.setText("P: " + str(self.current_value))
        except:
            pass

    def checkbox_click(self, value):
        self.is_selected = value

class ExtendedSuperSlider(ExtendedSlider):
    def __init__(self, joint, plugin_parent, parent = None):
        ExtendedSlider.__init__(self, joint, plugin_parent, parent)
        self.plugin_parent = plugin_parent
        self.position.close()
        self.target.setText("T: 0%")
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
        self.target.setText("T: " + str(value) + "%")

    def update(self):
        return

class JointSlider(ShadowGenericPlugin):
    """
    A generic class used to easily create new joint slider plugins, for the user
    to move the joints using sliders.
    """
    name = "Joint Slider"

    def __init__(self, joints_list):
        ShadowGenericPlugin.__init__(self)

        self.layout = QtGui.QHBoxLayout()
        self.layout.setAlignment(QtCore.Qt.AlignCenter)
        self.frame = QtGui.QFrame()

        #Add the sliders
        self.sliders = []
        for joint in joints_list:
            slider = ExtendedSlider(joint, self, self.frame)
            self.layout.addWidget(slider)
            self.sliders.append(slider)

        #Add a slider to control all the selected sliders
        selected_joints = Joint("Move Selected Joints", -100, 100)
        self.super_slider = ExtendedSuperSlider(selected_joints, self, self.frame)
        self.layout.addWidget(self.super_slider)

        self.frame.setLayout(self.layout)
        self.window.setWidget(self.frame)

    def sendupdate(self, dict):
        rospy.logerr("Virtual method, please implement.")

    def activate(self):
        ShadowGenericPlugin.activate(self)
        for slider in self.sliders:
            slider.timer.start(200)



class LightJointSlider(GenericPlugin):
    """
    A generic class used to easily create new joint slider plugins, for the user
    to move the joints using sliders.
    """
    name = "Joint Slider"

    def __init__(self):
        GenericPlugin.__init__(self)

        self.sliders = []
        self.super_slider = None

        self.layout = QtGui.QVBoxLayout()
        self.frame = QtGui.QFrame()

        self.control_frame = QtGui.QFrame()
        self.control_layout = QtGui.QHBoxLayout()
        self.control_frame.setLayout(self.control_layout)
        self.layout.addWidget(self.control_frame)

        self.sliders_layout = QtGui.QHBoxLayout()
        self.sliders_layout.setAlignment(QtCore.Qt.AlignCenter)
        self.sliders_frame = QtGui.QFrame()
        self.sliders_frame.setLayout(self.sliders_layout)

        self.layout.addWidget(self.sliders_frame)
        self.frame.setLayout(self.layout)

        self.window.setWidget(self.frame)

    def sendupdate(self, dict):
        rospy.logerr("Virtual method, please implement.")

    def activate(self, controller_type, joints_list):
        self.window.setWindowTitle(controller_type)
        GenericPlugin.activate(self)

        #Add the sliders
        self.sliders = []
        for joint in joints_list:
            slider = ExtendedSlider(joint, self, self.frame)
            self.sliders_layout.addWidget(slider)
            self.sliders.append(slider)

        #Add a slider to control all the selected sliders
        selected_joints = Joint("Move Selected Joints", -100, 100)
        self.super_slider = ExtendedSuperSlider(selected_joints, self, self.frame)
        self.layout.addWidget(self.super_slider)

        Qt.QTimer.singleShot(0, self.window.adjustSize)

        for slider in self.sliders:
            slider.timer.start(200)

    def on_close(self):
        for slider in self.sliders:
            slider.timer.stop()
            slider.setParent(None)
            self.sliders.remove(slider)
            del slider
        self.sliders = []
        if self.super_slider != None:
            self.super_slider.setParent(None)
            del self.super_slider
            self.super_slider = None

        GenericPlugin.on_close(self)



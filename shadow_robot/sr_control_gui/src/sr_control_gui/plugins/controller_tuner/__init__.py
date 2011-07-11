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

from generic_plugin import GenericPlugin
from sr_robot_msgs.srv import ForceController
from functools import partial

from PyQt4 import QtCore, QtGui, Qt


class JointPidSetter(QtGui.QFrame):
    """
    Set the force PID settings for a given joint.
    """

    def __init__(self, joint_name):
        """
        """
        QtGui.QFrame.__init__(self)
        self.joint_name = joint_name

        #/realtime_loop/change_force_PID_FFJ0
        service_name =  "/realtime_loop/change_force_PID_"+joint_name
        self.pid_service = rospy.ServiceProxy(service_name, ForceController)

        self.layout_ = QtGui.QHBoxLayout()

        label = QtGui.QLabel("<font color=red>"+joint_name+"</font>")
        self.layout_.addWidget( label )

        self.ordered_params = ["sgleftref",
                               "sgrightref",
                               "f",
                               "p",
                               "i",
                               "d",
                               "imax",
                               "deadband",
                               "sign" ]

        self.parameters = {}
        for param in self.ordered_params:
            #a parameter contains:
            #   - the value
            #   - a QLineEdit to be able to modify the value
            #   - an array containing the min/max
            self.parameters[param] = [0,0,[0,100]]

        for parameter_name in self.ordered_params:
            parameter = self.parameters[parameter_name]
            label = QtGui.QLabel(parameter_name)
            self.layout_.addWidget( label )

            text_edit = QtGui.QLineEdit()
            text_edit.setFixedHeight(30)
            text_edit.setFixedWidth(50)
            text_edit.setText( str(parameter[0]) )

            validator = QtGui.QIntValidator(parameter[2][0], parameter[2][1], self)
            text_edit.setValidator(validator)


            parameter[1] = text_edit
            self.layout_.addWidget(text_edit)
            plus_minus_frame = QtGui.QFrame()
            plus_minus_layout = QtGui.QVBoxLayout()

            plus_btn = QtGui.QPushButton()
            plus_btn.setText("+")
            plus_btn.setFixedWidth(25)
            plus_btn.setFixedHeight(25)
            plus_btn.clicked.connect(partial(self.plus, parameter_name))
            plus_minus_layout.addWidget(plus_btn)

            minus_btn = QtGui.QPushButton()
            minus_btn.setText("-")
            minus_btn.setFixedWidth(25)
            minus_btn.setFixedHeight(25)
            minus_btn.clicked.connect(partial(self.minus, parameter_name))
            plus_minus_layout.addWidget(minus_btn)

            plus_minus_frame.setLayout(plus_minus_layout)

            self.layout_.addWidget(plus_minus_frame)

        btn = QtGui.QPushButton()
        btn.setText("SET")
        btn.setToolTip("Sends the current PID parameters to the joint.")

        self.connect(btn, QtCore.SIGNAL('clicked()'),self.set_pid)
        self.layout_.addWidget(btn)

        self.setLayout(self.layout_)

    def plus(self, param_name):
        param = self.parameters[param_name]

        value = param[1].text().toInt()[0]
        value += 1
        if value > param[2][1]:
            value = param[2][1]

        param[1].setText( str( value ) )

    def minus(self, param_name):
        param = self.parameters[param_name]

        value = param[1].text().toInt()[0]
        value -= 1
        if value < param[2][0]:
            value = param[2][0]
        param[1].setText( str( value ) )


    def set_pid(self):
        for param in self.parameters.items():
            param[1][0] = param[1][1].text().toInt()
        try:
            self.pid_service(self.parameters["sgleftref"][0], self.parameters["sgrightref"][0],
                             self.parameters["f"][0], self.parameters["p"][0], self.parameters["i"][0],
                             self.parameters["d"][0], self.parameters["imax"][0],
                             self.parameters["deadband"][0], self.parameters["sign"][0] )
        except:
            print "Failed to set pid."


class FingerPIDSetter(QtGui.QFrame):
    """
    set the PID settings for the finger.
    """

    def __init__(self, finger_name, joint_names):
        QtGui.QFrame.__init__(self)

        self.setFrameShape(QtGui.QFrame.Box)

        self.finger_name = finger_name
        self.joint_names = joint_names

        self.layout_ = QtGui.QVBoxLayout()

        self.joint_pid_setter = []
        for joint_name in self.joint_names:
            self.joint_pid_setter.append( JointPidSetter(joint_name) )

        for j_pid_setter in self.joint_pid_setter:
            self.layout_.addWidget( j_pid_setter )

        self.setLayout(self.layout_)


class ControllerTuner(GenericPlugin):
    """
    A plugin to easily tune the force controller on the etherCAT hand.
    """
    name = "Controller Tuner"

    def __init__(self):
        GenericPlugin.__init__(self)

        self.frame = QtGui.QFrame()
        self.layout = QtGui.QVBoxLayout()

        self.joints = {"LF": ["LFJ0", "LFJ3", "LFJ4", "LFJ5"]}
        self.finger_pid_setters = []

        for finger in self.joints.items():
            self.finger_pid_setters.append( FingerPIDSetter(finger[0], finger[1]) )

        for f_pid_setter in self.finger_pid_setters:
            self.layout.addWidget( f_pid_setter )

        self.frame.setLayout(self.layout)
        self.window.setWidget(self.frame)

    def activate(self):
        GenericPlugin.activate(self)

        self.set_icon(self.parent.parent.rootPath + '/images/icons/iconHand.png')

    def on_close(self):
        GenericPlugin.on_close(self)
























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
import threading, time

from PyQt4 import QtCore, QtGui, Qt

class FullMovement(threading.Thread):
    def __init__(self, joint_name):
        threading.Thread.__init__(self)
        self.moving = False
        self.joint_name = joint_name
        self.movements = []

    def run(self):
        while(True):
            if self.moving == False:
                return
            else:
                print self.joint_name
                time.sleep(.1)

class AdvancedDialog(QtGui.QDialog):
    """
    Set the more advanced force PID parameters. We use a QDialog
    to keep the main GUI simple.
    """
    def __init__(self, parent, joint_name, parameters, ordered_params):
        QtGui.QDialog.__init__(self,parent)
        self.layout = QtGui.QVBoxLayout()

        self.ordered_params = ordered_params
        self.parameters = parameters

        frame = QtGui.QFrame()
        self.layout_ = QtGui.QHBoxLayout()
        for parameter_name in ordered_params:
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
        frame.setLayout(self.layout_)

        self.layout.addWidget(frame)

        self.btn_box = QtGui.QDialogButtonBox(self)
        self.btn_box.setOrientation(QtCore.Qt.Horizontal)
        self.btn_box.setStandardButtons(QtGui.QDialogButtonBox.Cancel|QtGui.QDialogButtonBox.Ok)
        self.layout.addWidget(self.btn_box)
        self.setLayout(self.layout)
        self.setWindowTitle("Advanced options: "+joint_name)

        QtCore.QObject.connect(self.btn_box, QtCore.SIGNAL("accepted()"), self.accept)
        QtCore.QObject.connect(self.btn_box, QtCore.SIGNAL("rejected()"), self.reject)
        QtCore.QMetaObject.connectSlotsByName(self)

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

    def getValues(self):
        for param in self.parameters.items():
            param[1][0] = param[1][1].text().toInt()[0]

        return self.parameters

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

        self.ordered_params = {"important":["f",
                                            "p",
                                            "i",
                                            "d",
                                            "imax"],
                               "advanced":["max_pwm",
                                           "sgleftref",
                                           "sgrightref",
                                           "deadband",
                                           "sign"]}

        self.important_parameters = {}
        for param in self.ordered_params["important"]:
            #a parameter contains:
            #   - the value
            #   - a QLineEdit to be able to modify the value
            #   - an array containing the min/max
            self.important_parameters[param] = [0,0,[-1023,1023]]

        self.advanced_parameters = {}
        for param in self.ordered_params["advanced"]:
            #a parameter contains:
            #   - the value
            #   - a QLineEdit to be able to modify the value
            #   - an array containing the min/max
            self.advanced_parameters[param] = [0,0,[-1023,1023]]

        self.advanced_parameters["max_pwm"][0] = 100

        for parameter_name in self.ordered_params["important"]:
            parameter = self.important_parameters[parameter_name]
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
            plus_btn.setFixedWidth(20)
            plus_btn.setFixedHeight(20)
            plus_btn.clicked.connect(partial(self.plus, parameter_name))
            plus_minus_layout.addWidget(plus_btn)

            minus_btn = QtGui.QPushButton()
            minus_btn.setText("-")
            minus_btn.setFixedWidth(20)
            minus_btn.setFixedHeight(20)
            minus_btn.clicked.connect(partial(self.minus, parameter_name))
            plus_minus_layout.addWidget(minus_btn)

            plus_minus_frame.setLayout(plus_minus_layout)

            self.layout_.addWidget(plus_minus_frame)

        btn = QtGui.QPushButton()
        btn.setText("SET")
        btn.setToolTip("Sends the current PID parameters to the joint.")
        self.connect(btn, QtCore.SIGNAL('clicked()'),self.set_pid)
        self.layout_.addWidget(btn)

        btn_advanced = QtGui.QPushButton()
        btn_advanced.setText("Advanced")
        btn_advanced.setToolTip("Set advanced parameters for this joint force controller.")
        self.connect(btn_advanced, QtCore.SIGNAL('clicked()'),self.advanced_options)
        self.layout_.addWidget(btn_advanced)

        self.moving = False
        self.full_movement = None

        self.btn_move = QtGui.QPushButton()
        self.btn_move.setText("Move")
        self.btn_move.setToolTip("Move the joint through a continuous movement, press again to stop.")
        self.connect(self.btn_move, QtCore.SIGNAL('clicked()'),self.move_clicked)
        self.layout_.addWidget(self.btn_move)

        self.setLayout(self.layout_)

    def plus(self, param_name):
        param = self.important_parameters[param_name]

        value = param[1].text().toInt()[0]
        value += 1
        if value > param[2][1]:
            value = param[2][1]

        param[1].setText( str( value ) )

    def minus(self, param_name):
        param = self.important_parameters[param_name]

        value = param[1].text().toInt()[0]
        value -= 1
        if value < param[2][0]:
            value = param[2][0]
        param[1].setText( str( value ) )

    def move_clicked(self):
        if self.moving:
            self.full_movement.moving = False
            self.moving = False
            self.full_movement.join()
            self.full_movement = None
            self.btn_move.setDown(False)
        else:
            self.moving = True
            self.full_movement = FullMovement(self.joint_name)
            self.full_movement.moving = True
            self.full_movement.start()
            self.btn_move.setDown(True)

    def set_pid(self):
        for param in self.important_parameters.items():
            param[1][0] = param[1][1].text().toInt()[0]
        #try:
        self.pid_service(self.advanced_parameters["max_pwm"][0], self.advanced_parameters["sgleftref"][0],
                         self.advanced_parameters["sgrightref"][0], self.important_parameters["f"][0],
                         self.important_parameters["p"][0], self.important_parameters["i"][0],
                         self.important_parameters["d"][0], self.important_parameters["imax"][0],
                         self.advanced_parameters["deadband"][0], self.advanced_parameters["sign"][0] )
        #except:
        #    print "Failed to set pid."

    def advanced_options(self):
        adv_dial = AdvancedDialog(self, self.joint_name, self.advanced_parameters,
                                  self.ordered_params["advanced"])
        if adv_dial.exec_():
            modified_adv_param = adv_dial.getValues()
            for param in modified_adv_param.items():
                self.advanced_parameters[param[0]] = param[1]

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

        self.joints = {"FF": ["FFJ0", "FFJ3", "FFJ4"],
                       "MF": ["MFJ0", "MFJ3", "MFJ4"],
                       "RF": ["RFJ0", "RFJ3", "RFJ4"],
                       "LF": ["LFJ0", "LFJ3", "LFJ4", "LFJ5"],
                       "TH": ["THJ1", "THJ2", "THJ3", "THJ4", "THJ5"]}
        self.finger_pid_setters = []

        for finger in self.joints.items():
            self.finger_pid_setters.append( FingerPIDSetter(finger[0], finger[1]) )

        self.qtab_widget = QtGui.QTabWidget()
        for f_pid_setter in self.finger_pid_setters:
            self.qtab_widget.addTab(f_pid_setter, f_pid_setter.finger_name)

        self.layout.addWidget( self.qtab_widget )

        self.frame.setLayout(self.layout)
        self.window.setWidget(self.frame)

    def activate(self):
        GenericPlugin.activate(self)

        self.set_icon(self.parent.parent.rootPath + '/images/icons/iconHand.png')

    def on_close(self):
        GenericPlugin.on_close(self)
























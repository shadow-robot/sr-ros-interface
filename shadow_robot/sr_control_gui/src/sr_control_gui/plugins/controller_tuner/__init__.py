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
from sr_robot_msgs.srv import ForceController, SetEffortControllerGains, SetMixedPositionVelocityPidGains, SetPidGains
from functools import partial
import threading, time, math

from std_msgs.msg import Float64

from PyQt4 import QtCore, QtGui, Qt

from pr2_mechanism_msgs.srv import ListControllers
from controller_tuner.automatic_procedures import RunGA, RunFriction, FullMovement

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

            validator = QtGui.QDoubleValidator(parameter[2][0], parameter[2][1], 4, self)
            text_edit.setValidator(validator)

            parameter[1] = text_edit
            self.layout_.addWidget(text_edit)
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

    def getValues(self):
        for param in self.parameters.items():
            param[1][0] = param[1][1].text().toDouble()[0]

        return self.parameters

class JointPidSetter(QtGui.QFrame):
    """
    Set the force PID settings for a given joint.
    """

    def __init__(self, joint_name, ordered_params, controller_type, parent):
        """
        """
        QtGui.QFrame.__init__(self)

        self.ordered_params = ordered_params
        self.controller_type = controller_type

        self.joint_name = joint_name
        self.parent = parent

        self.friction = False
        self.friction_thread = None

        self.layout_ = QtGui.QHBoxLayout()

        self.frame_important_parameters = QtGui.QFrame()
        self.layout_important_param = QtGui.QHBoxLayout()
        self.frame_important_parameters.setLayout(self.layout_important_param)

        self.layout_.addWidget(self.frame_important_parameters)

        btn = QtGui.QPushButton()
        btn.setText("SET")
        btn.setToolTip("Sends the current PID parameters to the joint.")
        self.connect(btn, QtCore.SIGNAL('clicked()'),self.set_pid)
        self.layout_.addWidget(btn)

        self.btn_advanced = QtGui.QPushButton()
        self.btn_advanced.setText("Advanced")
        self.btn_advanced.setToolTip("Set advanced parameters for this joint force controller.")
        self.connect(self.btn_advanced, QtCore.SIGNAL('clicked()'),self.advanced_options)
        self.layout_.addWidget(self.btn_advanced)

        self.btn_automatic_pid = QtGui.QPushButton()
        self.btn_automatic_pid.setText( "Automatic" )
        self.btn_automatic_pid.setToolTip("Finishes the PID tuning automatically using a genetic algorithm.")
        self.connect(self.btn_automatic_pid, QtCore.SIGNAL('clicked()'),self.automatic_tuning)
        self.layout_.addWidget(self.btn_automatic_pid)
        self.btn_automatic_pid.setEnabled(False)

        self.btn_friction_compensation = QtGui.QPushButton()
        self.btn_friction_compensation.setText( "Friction" )
        self.btn_friction_compensation.setToolTip("Computes the Friction Compensation umap")
        self.connect(self.btn_friction_compensation, QtCore.SIGNAL('clicked()'),self.friction_compensation)
        self.layout_.addWidget(self.btn_friction_compensation)
        self.btn_friction_compensation.setEnabled(False)

        self.tuning = False
        self.GA_thread = None

        self.moving = False
        self.full_movement = None

        self.btn_move = QtGui.QPushButton()
        self.btn_move.setText("Move")
        self.btn_move.setToolTip("Move the joint through a continuous movement, press again to stop.")
        self.connect(self.btn_move, QtCore.SIGNAL('clicked()'),self.move_clicked)
        if self.controller_type == "Motor Force":
            self.btn_move.setEnabled(False)
        self.layout_.addWidget(self.btn_move)

        self.setLayout(self.layout_)

    def friction_compensation(self):
        if self.friction:
            self.btn_friction_compensation.setIcon(self.green_icon)
            self.friction_thread.tuning = False
            self.friction_thread.FC.stop()
            self.friction_thread.join()
            self.friction_thread = None

            self.btn_friction_compensation.setIcon(self.green_icon)
            self.friction = False
            self.btn_move.setEnabled(True)

        else:
            if self.moving:
                self.full_movement.moving = False
                self.moving = False
                self.full_movement.join()
                self.full_movement = None
                self.btn_move.setIcon(self.green_icon)
                self.btn_move.setEnabled(False)

            self.friction_thread = RunFriction(self.joint_name, self)
            self.friction_thread.start()

            self.friction = True
            self.btn_friction_compensation.setIcon(self.red_icon)

    def friction_finished(self):
        #TODO: add a popup to tell people the friction finished properly,
        # may be we should display the points + the line
        self.btn_friction_compensation.setIcon(self.green_icon)
        self.friction_thread.tuning = False
        self.friction_thread.FC.stop()
        self.friction_thread = None

        self.btn_friction_compensation.setIcon(self.green_icon)
        self.friction = False
        self.btn_move.setEnabled(True)

    def move_clicked(self):
        if self.moving:
            self.full_movement.moving = False
            self.moving = False
            self.full_movement.join()
            self.full_movement = None
            self.btn_move.setIcon(self.green_icon)
        else:
            self.moving = True
            self.full_movement = FullMovement(self.joint_name, self.controller_type)
            self.full_movement.moving = True
            self.full_movement.start()
            self.btn_move.setIcon(self.red_icon)

    def automatic_tuning(self):
        if self.tuning:
            self.ga_stopped()
        else:
            if self.moving:
                self.full_movement.moving = False
                self.moving = False
                self.full_movement.join()
                self.full_movement = None
                self.btn_move.setIcon(self.green_icon)
                self.btn_move.setEnabled(False)

            aut_tuning_dialog = AutomaticTuningDialog( self, self.joint_name,
                                                       self.important_parameters,
                                                       self.advanced_parameters)
            if aut_tuning_dialog.exec_():
                pid_settings = aut_tuning_dialog.getValues()

                self.GA_thread = RunGA(self.joint_name, pid_settings, self)
                self.GA_thread.start()

                self.tuning = True
                self.btn_automatic_pid.setIcon(self.red_icon)

    def ga_stopped(self, result = None):
        if result is not None:
            ch = result["best_chromosome"]
            #QtGui.QMessageBox.information( self,
            #                               "Successfully tuned the joint: " + self.joint_name,
            #                               " best fitness: " + str( result["best_fitness"]) +" for: [P:"+str( ch["p"] )+" I:"+str( ch["i"] ) +" D:"+str( ch["d"] )+" Imax:"+str( ch["imax"] )+"]" )

            rospy.loginfo( "Successfully tuned the joint: " + self.joint_name + " best fitness: " + str( result["best_fitness"]) +" for: [P:"+str( ch["p"] )+" I:"+str( ch["i"] ) +" D:"+str( ch["d"] )+" Imax:"+str( ch["imax"] )+"]" )

            #use the best parameters
            self.important_parameters["p"][0] = ch["p"]
            self.important_parameters["p"][1].setText(str( ch["p"]) )
            self.important_parameters["i"][0] = ch["i"]
            self.important_parameters["i"][1].setText(str( ch["i"]) )
            self.important_parameters["d"][0] = ch["d"]
            self.important_parameters["d"][1].setText(str( ch["d"]) )
            self.important_parameters["imax"][0] = ch["imax"]
            self.important_parameters["imax"][1].setText(str( ch["imax"]) )
            self.set_pid()

        self.GA_thread.tuning = False
        self.GA_thread.robot_lib.stopped = True
        if result is None:
            self.GA_thread.join()
        self.GA_thread = None

        self.btn_automatic_pid.setIcon(self.green_icon)
        self.tuning = False
        self.btn_move.setEnabled(True)


    def set_pid(self):
        for param in self.important_parameters.items():
            param[1][0] = param[1][1].text().toInt()[0]

        if self.controller_type == "Motor Force":
            self.pid_service(int(self.advanced_parameters["max_pwm"][0]), int(self.advanced_parameters["sgleftref"][0]),
                             int(self.advanced_parameters["sgrightref"][0]), int(self.important_parameters["f"][0]),
                             int(self.important_parameters["p"][0]), int(self.important_parameters["i"][0]),
                             int(self.important_parameters["d"][0]), int(self.important_parameters["imax"][0]),
                             int(self.advanced_parameters["deadband"][0]), int(self.advanced_parameters["sign"][0]) )
        elif self.controller_type == "Position" or self.controller_type == "Velocity":
            self.pid_service(int(self.important_parameters["p"][0]), int(self.important_parameters["i"][0]),
                             int(self.important_parameters["d"][0]), int(self.important_parameters["i_clamp"][0]),
                             int(self.advanced_parameters["max_force"][0]), float(self.advanced_parameters["deadband"][0]),
                             int(self.advanced_parameters["friction_deadband"][0]) )
        elif self.controller_type == "Mixed Position/Velocity":
            self.pid_service(float(self.important_parameters["p"][0]), float(self.important_parameters["i"][0]),
                             float(self.important_parameters["d"][0]), float(self.important_parameters["i_clamp"][0]),
                             float(self.advanced_parameters["max_force"][0]), float(self.advanced_parameters["min_velocity"][0]),
                             float(self.advanced_parameters["max_velocity"][0]), float(self.advanced_parameters["velocity_slope"][0]),
                             float(self.advanced_parameters["position_deadband"][0]), int(self.advanced_parameters["friction_deadband"][0]) )
        elif self.controller_type == "Effort":
            try:
                self.pid_service(int(self.important_parameters["max_force"][0]), int(self.important_parameters["friction_deadband"][0]) )
            except rospy.ServiceException, e:
                print "Service did not process request: %s"%str(e)
        else:
            print "", self.controller_type, " is not a recognized controller type."

    def advanced_options(self):
        adv_dial = AdvancedDialog(self, self.joint_name, self.advanced_parameters,
                                  self.ordered_params["advanced"])
        if adv_dial.exec_():
            modified_adv_param = adv_dial.getValues()
            for param in modified_adv_param.items():
                self.advanced_parameters[param[0]] = param[1]

    def activate(self):
        #use the correct service
        self.pid_service = None
        if self.controller_type == "Motor Force":
            #/realtime_loop/change_force_PID_FFJ0
            service_name =  "/realtime_loop/change_force_PID_"+self.joint_name.upper()
            self.pid_service = rospy.ServiceProxy(service_name, ForceController)

        elif self.controller_type == "Position":
            #/sh_ffj3_position_controller/set_gains
            service_name =  "/sh_"+self.joint_name.lower()+"_position_controller/set_gains"
            self.pid_service = rospy.ServiceProxy(service_name, SetPidGains)

        elif self.controller_type == "Velocity":
            #/sh_ffj3_velocity_controller/set_gains
            service_name =  "/sh_"+self.joint_name.lower()+"_velocity_controller/set_gains"
            self.pid_service = rospy.ServiceProxy(service_name, SetPidGains)

        elif self.controller_type == "Mixed Position/Velocity":
            #/sh_ffj3_mixed_position_velocity_controller/set_gains
            service_name =  "/sh_"+self.joint_name.lower()+"_mixed_position_velocity_controller/set_gains"
            self.pid_service = rospy.ServiceProxy(service_name, SetMixedPositionVelocityPidGains)

        elif self.controller_type == "Effort":
            #/sh_ffj3_effort_controller/set_gains
            service_name =  "/sh_"+self.joint_name.lower()+"_effort_controller/set_gains"
            self.pid_service = rospy.ServiceProxy(service_name, SetEffortControllerGains)

        else:
            print "", self.controller_type, " is not a recognized controller type."

        label = QtGui.QLabel("<font color=red>"+self.joint_name+"</font>")
        self.layout_important_param.addWidget( label )

        self.important_parameters = {}
        for param in self.ordered_params["important"]:
            #a parameter contains:
            #   - the value
            #   - a QLineEdit to be able to modify the value
            #   - an array containing the min/max
            self.important_parameters[param] = [0,0,[-65535,65535]]

        self.advanced_parameters = {}
        if self.ordered_params.has_key("advanced"):
            self.btn_advanced.setEnabled(True)
            for param in self.ordered_params["advanced"]:
                #a parameter contains:
                #   - the value
                #   - a QLineEdit to be able to modify the value
                #   - an array containing the min/max
                self.advanced_parameters[param] = [0,0,[-65535,65535]]

            if self.advanced_parameters.has_key("max_pwm"):
                self.advanced_parameters["max_pwm"][0] = 1023
            if self.advanced_parameters.has_key("max_force"):
                self.advanced_parameters["max_force"][0] = 1000
            if self.advanced_parameters.has_key("min_velocity"):
                self.advanced_parameters["min_velocity"][0] = -0.5
            if self.advanced_parameters.has_key("min_velocity"):
                self.advanced_parameters["min_velocity"][0] = 0.5
            if self.advanced_parameters.has_key("velocity_slope"):
                self.advanced_parameters["velocity_slope"][0] = 5.0

        else:
            self.btn_advanced.setEnabled(False)

        for parameter_name in self.ordered_params["important"]:
            parameter = self.important_parameters[parameter_name]
            label = QtGui.QLabel(parameter_name)
            self.layout_important_param.addWidget( label )

            text_edit = QtGui.QLineEdit()
            text_edit.setFixedHeight(30)
            text_edit.setFixedWidth(50)
            text_edit.setText( str(parameter[0]) )

            validator = QtGui.QIntValidator(parameter[2][0], parameter[2][1], self)
            text_edit.setValidator(validator)


            parameter[1] = text_edit
            self.layout_important_param.addWidget(text_edit)

        self.green_icon = QtGui.QIcon(self.parent.parent.parent.parent.rootPath + '/images/icons/colors/green.png')
        self.red_icon = QtGui.QIcon(self.parent.parent.parent.parent.rootPath + '/images/icons/colors/red.png')
        self.btn_move.setIcon(self.green_icon)
        self.btn_automatic_pid.setIcon(self.green_icon)

        Qt.QTimer.singleShot(0, self.adjustSize)
        Qt.QTimer.singleShot(0, self.parent.parent.window.adjustSize)



    def on_close(self):
        if self.full_movement != None:
            self.full_movement.moving = False
            self.moving = False
            self.full_movement.join()

        if self.GA_thread != None:
            self.GA_thread.tuning = False
            self.GA_thread.robot_lib.stopped = True
            self.GA_thread.join()
            self.GA_thread = None

class FingerPIDSetter(QtGui.QFrame):
    """
    set the PID settings for the finger.
    """

    def __init__(self, finger_name, joint_names, ordered_params, controller_type, parent):
        QtGui.QFrame.__init__(self)
        self.parent = parent
        self.setFrameShape(QtGui.QFrame.Box)

        self.finger_name = finger_name
        self.joint_names = joint_names

        self.layout_ = QtGui.QVBoxLayout()

        self.joint_pid_setter = []
        for joint_name in self.joint_names:
            self.joint_pid_setter.append( JointPidSetter(joint_name, ordered_params, controller_type, self) )

        for j_pid_setter in self.joint_pid_setter:
            self.layout_.addWidget( j_pid_setter )

        self.setLayout(self.layout_)

        for jps in self.joint_pid_setter:
            jps.activate()

    def on_close(self):
        for j_pid_setter in self.joint_pid_setter:
            j_pid_setter.on_close()


class ControllerTuner(GenericPlugin):
    """
    A plugin to easily tune the controllers on the etherCAT hand.
    """
    name = "Controllers Tuner"

    def __init__(self):
        GenericPlugin.__init__(self)
        self.frame = QtGui.QFrame()
        self.layout = QtGui.QVBoxLayout()

        self.frame_controller_type = QtGui.QFrame()
        self.layout_controller_type = QtGui.QHBoxLayout()

        #add a combo box to select the controller type
        self.label_control = QtGui.QLabel("Controller type: ")
        self.layout_controller_type.addWidget(self.label_control)
        self.controller_combo_box = QtGui.QComboBox(self.frame_controller_type)
        self.controller_combo_box.setToolTip("Choose the type of controller you want to tune.")

        self.all_controller_types = ["Motor Force", "Position", "Velocity",
                                     "Mixed Position/Velocity", "Effort"]
        self.ordered_controller_types = ["Motor Force", "Position", "Velocity",
                                         "Mixed Position/Velocity", "Effort"]
        self.controller_params = {"Motor Force": { "important":["f",
                                                                "p",
                                                                "i",
                                                                "d",
                                                                "imax"],
                                                   "advanced":["max_pwm",
                                                               "sgleftref",
                                                               "sgrightref",
                                                               "deadband",
                                                               "sign"] },

                                  "Position":{ "important":[ "p",
                                                             "i",
                                                             "d",
                                                             "i_clamp"],
                                               "advanced":["max_force",
                                                           "deadband",
                                                           "friction_deadband"] },

                                  "Velocity":{ "important":[ "p",
                                                             "i",
                                                             "d",
                                                             "i_clamp"],
                                               "advanced":["max_force",
                                                           "deadband",
                                                           "friction_deadband"] },

                                  "Mixed Position/Velocity":{ "important":[ "p",
                                                                            "i",
                                                                            "d",
                                                                            "i_clamp"],
                                                              "advanced":["max_force",
                                                                          "min_velocity",
                                                                          "max_velocity",
                                                                          "velocity_slope",
                                                                          "position_deadband",
                                                                          "friction_deadband"] },

                                  "Effort":{ "important":["max_force", "friction_deadband"]}}

        self.frame_controller_type.connect(self.controller_combo_box, QtCore.SIGNAL('activated(int)'), self.changed_controller_type)
        self.layout_controller_type.addWidget(self.controller_combo_box)
        self.frame_controller_type.setLayout(self.layout_controller_type)

        self.refresh_btn = QtGui.QPushButton()
        self.refresh_btn.setText("Refresh Controllers")
        self.refresh_btn.setToolTip("Check with the controller manager to see which controller is currently running")
        self.frame_controller_type.connect(self.refresh_btn, QtCore.SIGNAL('clicked()'),self.refresh_available_controllers)
        self.layout_controller_type.addWidget(self.refresh_btn)

        self.layout.addWidget(self.frame_controller_type)

        self.refresh_available_controllers()

        self.finger_pid_setters = []
        self.qtab_widget = QtGui.QTabWidget()
        self.layout.addWidget( self.qtab_widget )

        self.frame.setLayout(self.layout)
        self.window.setWidget(self.frame)


    def refresh_available_controllers(self):
        rospy.wait_for_service('/pr2_controller_manager/list_controllers')
        controllers = rospy.ServiceProxy('/pr2_controller_manager/list_controllers', ListControllers)
        resp = None
        try:
            resp = controllers()
        except rospy.ServiceException, e:
            print "Service did not process request: %s"%str(e)

        self.controller_type = "default"
        controllers_tmp = []
        self.ordered_controller_types = []
        self.controller_combo_box.clear()
        self.controller_combo_box.addItem("Motor Force")
        self.ordered_controller_types.append("Motor Force")
        if resp != None:
            for state,controller in zip(resp.state,resp.controllers):
                if state == "running":
                    split = controller.split("_")
                    ctrl_type_tmp = split[2]
                    for defined_ctrl_type in self.all_controller_types:
                        if ctrl_type_tmp.lower() in defined_ctrl_type.lower():
                            self.controller_combo_box.addItem(defined_ctrl_type)
                            self.ordered_controller_types.append(defined_ctrl_type)
                            return

    def changed_controller_type(self, index):
        self.joints = {"FF": ["FFJ0", "FFJ3", "FFJ4"],
                       "MF": ["MFJ0", "MFJ3", "MFJ4"],
                       "RF": ["RFJ0", "RFJ3", "RFJ4"],
                       "LF": ["LFJ0", "LFJ3", "LFJ4", "LFJ5"],
                       "TH": ["THJ1", "THJ2", "THJ3", "THJ4", "THJ5"]}

        for fps in self.finger_pid_setters:
            fps.setParent(None)
            del fps
        self.finger_pid_setters = []

        controller_type = self.ordered_controller_types[index]
        params = self.controller_params[ controller_type ]
        for finger in self.joints.items():
            self.finger_pid_setters.append( FingerPIDSetter(finger[0], finger[1], params, controller_type, self) )
        for f_pid_setter in self.finger_pid_setters:
            self.qtab_widget.addTab(f_pid_setter, f_pid_setter.finger_name)

        Qt.QTimer.singleShot(0, self.window.adjustSize)

    def activate(self):
        GenericPlugin.activate(self)

        self.changed_controller_type(0)
        self.set_icon(self.parent.parent.rootPath + '/images/icons/iconHand.png')

    def on_close(self):
        GenericPlugin.on_close(self)
        for f_pid_setter in self.finger_pid_setters:
            f_pid_setter.on_close()
























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

#from sr_friction_compensation.u_map_computation_main import FrictionCompensation
#from sr_friction_compensation.lib.ethercat_robot_lib import EtherCAT_Robot_Lib

from PyQt4 import QtCore, QtGui, Qt
from functools import partial
import threading, time, math

from std_msgs.msg import Float64



class AutomaticTuningDialog(QtGui.QDialog):
    """
    Set the parameters for the automatic pid tuner.
    """
    def __init__(self, parent, joint_name, parameters, hidden_parameters):
        QtGui.QDialog.__init__(self,parent)
        self.layout = QtGui.QVBoxLayout()

        self.ordered_parameters = [["population size", "int", 2, 30 ], ["number of generations", "int", 5, 100],["percentage of mutation", "float", 0.0, 1.0],
                                   ["P_min", "int", 0, 10000], ["P_max", "int", 0, 10000],
                                   ["I_min", "int", 0, 10000], ["I_max", "int", 0, 10000],
                                   ["D_min", "int", 0, 10000], ["D_max", "int", 0, 10000],
                                   ["Imax_min", "int", 0, 10000], ["Imax_max", "int", 0, 10000],
                                   ["sign", "hidden"], ["max_pwm", "hidden"]]

        min_max = self.compute_min_max( parameters, ["p", "i", "d", "imax"])

        self.parameters = {"population size":10, "number of generations":7,"percentage of mutation":0.25,
                           "P_min":min_max["p"][0], "P_max": min_max["p"][1],
                           "I_min":min_max["i"][0], "I_max": min_max["i"][1],
                           "D_min":min_max["d"][0], "D_max": min_max["d"][1],
                           "Imax_min":min_max["imax"][0], "Imax_max": min_max["imax"][1],
                           "sign": hidden_parameters["sign"][0],
                           "max_pwm": hidden_parameters["max_pwm"][0]
                           }
        frame_ga = QtGui.QFrame()
        frame_min_max = QtGui.QFrame()
        layout_ga = QtGui.QHBoxLayout()
        layout_min_max = QtGui.QHBoxLayout()

        self.text_edits = []
        for param in self.ordered_parameters:
            if param[1] == "hidden":
                continue

            label = QtGui.QLabel(param[0])

            text_edit = QtGui.QLineEdit()
            text_edit.setFixedHeight(30)
            text_edit.setFixedWidth(50)

            text_edit.setText( str(self.parameters[ param[0] ]) )

            if param[1] == "int":
                validator = QtGui.QIntValidator(param[2], param[3], self)
            elif param[1] == "float":
                print param, " -> float"
                validator = QtGui.QDoubleValidator(param[2], param[3], 2, self)
            text_edit.setValidator(validator)

            self.text_edits.append(text_edit)

            if "_min" in param[0] or "_max" in param[0]:
                layout_min_max.addWidget(label)
                layout_min_max.addWidget(text_edit)
            else:
                layout_ga.addWidget(label)
                layout_ga.addWidget(text_edit)

        frame_ga.setLayout(layout_ga)
        self.layout.addWidget(frame_ga)

        frame_min_max.setLayout(layout_min_max)
        self.layout.addWidget(frame_min_max)

        self.btn_box = QtGui.QDialogButtonBox(self)
        self.btn_box.setOrientation(QtCore.Qt.Horizontal)
        self.btn_box.setStandardButtons(QtGui.QDialogButtonBox.Cancel|QtGui.QDialogButtonBox.Ok)
        self.layout.addWidget(self.btn_box)
        self.setLayout(self.layout)
        self.setWindowTitle("Automatic PID tuning: "+joint_name)

        QtCore.QObject.connect(self.btn_box, QtCore.SIGNAL("accepted()"), self.accept)
        QtCore.QObject.connect(self.btn_box, QtCore.SIGNAL("rejected()"), self.reject)
        QtCore.QMetaObject.connectSlotsByName(self)

    def compute_min_max(self, parameters, names):
        min_max = {}
        for name in names:
            mini = max( parameters[name][0] - 500  , 0 )
            maxi = min( parameters[name][0] + 500  , 10000 )
            min_max[name] = [mini, maxi]
        return min_max

    def getValues(self):
        for param,text_edit in zip(self.ordered_parameters, self.text_edits):
            if param[1] == "int" or param[1] == "hidden":
                self.parameters[ param[0] ] = text_edit.text().toInt()[0]
            elif param[1] == "float":
                self.parameters[ param[0] ] = text_edit.text().toDouble()[0]
        return self.parameters

class BaseMovement(object):
    topic_based_on_controller_type = {"Motor Force":None,
                                      "Position":"_position_controller/command",
                                      "Velocity":"_velocity_controller/command",
                                      "Mixed Position/Velocity":"_mixed_position_velocity_controller/command",
                                      "Effort":"_effort_controller/command"}
    def __init__(self, joint_name, controller_type):
        self.joint_name = joint_name

        self.msg_to_send = Float64()
        self.msg_to_send.data = 0.0
        self.sleep_time = 0.0001

        topic = "/sh_"+ joint_name.lower() + self.topic_based_on_controller_type[controller_type]
        self.publisher = rospy.Publisher(topic, Float64)

    def publish(self, mvt_percentage):
        self.update(mvt_percentage)
        self.publisher.publish(self.msg_to_send)
        time.sleep(self.sleep_time)

    def update(self, mvt_percentage):
        pass

class SinusoidMovement(BaseMovement):
    def __init__(self, joint_name, amplitude = 400, offset = 0, controller_type = None):
        BaseMovement.__init__(self,joint_name, controller_type)
        self.offset = offset
        self.amplitude = amplitude

    def update(self, mvt_percentage):
        value = float(self.amplitude) * math.sin(2.0*3.14159 * float(mvt_percentage)/100.) + float(self.offset)
        self.msg_to_send.data = value

class StepMovement(BaseMovement):
    def __init__(self, joint_name, amplitude = 400, nb_steps = 50, offset = 0.0, controller_type = None):
        BaseMovement.__init__(self,joint_name, controller_type)
        self.amplitude = amplitude
        self.nb_steps  = nb_steps

        self.steps = []
        for i in range(0, int(nb_steps/2)):
            self.steps.append(2.*float(i)*float(self.amplitude / nb_steps) - self.amplitude / 2.0 + offset)
        reversed_steps = self.steps[:]
        reversed_steps.reverse()
        self.steps += reversed_steps

    def update(self, mvt_percentage):
        index = int(self.nb_steps * mvt_percentage / 100.)
        if index >= len(self.steps):
            index = len(self.steps) - 1
        value = self.steps[ index ]
        self.msg_to_send.data = value


class RunGA(threading.Thread):
    def __init__(self, joint_name, parameters, parent):
        threading.Thread.__init__(self)

        self.parent = parent

        self.joint_name = joint_name
        self.tuning = True
        self.parameters = parameters

    def run(self):
        pass


class RunFriction(threading.Thread):
    def __init__(self, joint_name, parent):
        threading.Thread.__init__(self)
        self.parent = parent
        self.joint_name = joint_name
        self.stopped = False
        #self.robot_lib = EtherCAT_Robot_Lib(joint_name)
        #self.FC = FrictionCompensation(joint_name = joint_name, n = 15,P=0, I=0, D=0, shift=0, lib = self.robot_lib)

    def run(self):
        #self.FC.run()
        self.parent.friction_finished()


class FullMovement(threading.Thread):
    def __init__(self, joint_name, controller_type):
        threading.Thread.__init__(self)


        self.possible_movements = {"Motor Force": None,
                                   "Position":[ StepMovement(joint_name, amplitude = 0.1, offset= 0.3, nb_steps = 100, controller_type = "Position"),
                                                SinusoidMovement(joint_name, amplitude = 0.1, offset = 0.3, controller_type = "Position"),
                                                StepMovement(joint_name, amplitude=0.1, offset = 0.3, nb_steps = 10, controller_type = "Position")],
                                   "Velocity":[ StepMovement(joint_name, amplitude = 0.3, nb_steps = 100, controller_type = "Velocity"),
                                                SinusoidMovement(joint_name, amplitude = 0.1, offset = 0.0, controller_type = "Velocity"),
                                                StepMovement(joint_name, amplitude=0.5, nb_steps = 10, controller_type = "Velocity")],
                                   "Mixed Position/Velocity": [ StepMovement(joint_name, amplitude = 0.3, nb_steps = 100,  controller_type = "Mixed Position/Velocity"),
                                                                SinusoidMovement(joint_name, amplitude = 0.7, offset = 0.7, controller_type = "Mixed Position/Velocity"),
                                                                StepMovement(joint_name, amplitude=0.5, nb_steps = 10, controller_type = "Mixed Position/Velocity")],
                                   "Effort": [StepMovement(joint_name, amplitude=1200, nb_steps = 10, controller_type = "Effort"),
                                              SinusoidMovement(joint_name, amplitude = 400, offset = 0, controller_type = "Effort"),
                                              StepMovement(joint_name, amplitude = 500, nb_steps = 100, controller_type = "Effort") ] }




        self.moving = False
        self.joint_name = joint_name
        self.iterations = 10000
        self.movements = self.possible_movements[controller_type]

    def run(self):
        while(True):
            for movement in self.movements:
                for mvt_percentage in range(0, self.iterations):
                    if self.moving == False:
                        return
                    else:
                        movement.publish(mvt_percentage/(self.iterations/100.))

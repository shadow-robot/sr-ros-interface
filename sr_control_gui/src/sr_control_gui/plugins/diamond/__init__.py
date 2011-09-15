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

import threading, math, time
from functools import partial
from config import Config
from generic_plugin import GenericPlugin
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class BaseMovement(object):
    def __init__(self, finger_name):
        self.finger_name = finger_name

        self.msg_to_send_j3 = Float64()
        self.msg_to_send_j3.data = 0.0

        self.msg_to_send_j4 = Float64()
        self.msg_to_send_j4.data = 0.0

        self.sleep_time = 0.001

        topic = "/sh_"+ finger_name.lower() +"j3_position_controller/command"
        self.publisher_j3 = rospy.Publisher(topic, Float64)
        topic = "/sh_"+ finger_name.lower() +"j4_position_controller/command"
        self.publisher_j4 = rospy.Publisher(topic, Float64)


    def publish(self, mvt_percentage):
        result = self.update(mvt_percentage)
        self.publisher_j3.publish(self.msg_to_send_j3)
        self.publisher_j4.publish(self.msg_to_send_j4)
        time.sleep(self.sleep_time)
        return result

    def update(self, mvt_percentage):
        pass

    def close(self):
        if self.publisher_j3 != None:
            self.publisher_j3.unregister()
            self.publisher_j3 = None
        if self.publisher_j4 != None:
            self.publisher_j4.unregister()
            self.publisher_j4 = None

class BaseMovementWithLatching(BaseMovement):
    def __init__(self, finger_name, epsilon = 0.03):
        BaseMovement.__init__(self, finger_name)

        self.mutex = threading.Lock()

        self.subscriber = rospy.Subscriber("/srh/position/joint_states", JointState, self.callback)
        self.indexes_in_joint_states = []
        self.epsilon = epsilon

        self.wait_counter = 0
        self.j3_target_ok = True
        self.j4_target_ok = True

    def callback(self, msg):
        if self.mutex.acquire(False):
            if len(self.indexes_in_joint_states) == 0:
                for index,name in enumerate(msg.name):
                    if self.finger_name.upper()+"J3" == name:
                        self.indexes_in_joint_states.append(index)
                    elif self.finger_name.upper()+"J4" == name:
                        self.indexes_in_joint_states.append(index)

                if (self.msg_to_send_j3.data - msg.position[self.indexes_in_joint_states[0]]) < self.epsilon:
                    self.j3_target_ok = True
                if (self.msg_to_send_j4.data - msg.position[self.indexes_in_joint_states[0]]) < self.epsilon:
                    self.j4_target_ok = True
            self.wait_counter += 1
            self.mutex.release()


class EllipsoidMovement(BaseMovementWithLatching):
    def __init__(self, finger_name, amplitude_j3 = 0.15, amplitude_j4 = 0.3, offset_j3 = 0.78, offset_j4 = 0.0, epsilon = 0.03):
        BaseMovementWithLatching.__init__(self,finger_name, epsilon)
        self.amplitude_j3 = amplitude_j3
        self.amplitude_j4 = amplitude_j4
        self.offset_j3 = offset_j3
        self.offset_j4 = offset_j4

    def update(self, mvt_percentage):
        #get the mutex
        trying_mutex_iteration = 0
        while not self.mutex.acquire(False):
            time.sleep(0.001)
            trying_mutex_iteration += 1
            if trying_mutex_iteration > 1000:
                return False

        if (self.j3_target_ok and self.j4_target_ok) or (self.wait_counter >= 1):
            self.wait_counter = 0
            self.j3_target_ok = False
            self.j4_target_ok = False
            j3 = self.amplitude_j3 * math.sin(2.0*3.14159 * mvt_percentage) + self.offset_j3
            j4 = self.amplitude_j4 * math.cos(2.0*3.14159 * mvt_percentage) + self.offset_j4

            print " j3: ", j3, " j4: ", j4

            self.msg_to_send_j3.data = j3
            self.msg_to_send_j4.data = j4

            self.mutex.release()
            return True
        self.mutex.release()
        return False


class Movement(threading.Thread):
    def __init__(self, finger_name):
        threading.Thread.__init__(self)
        self.moving = False
        self.finger_name = finger_name
        self.iterations_max = 1000
        self.movements = []

        for i in range(1, 5):
            self.movements.append( EllipsoidMovement(finger_name, amplitude_j3=(float(i)/10.0), amplitude_j4=(float(i)/20.0)) )
        for i in range(0, 4):
            self.movements.append( EllipsoidMovement(finger_name, amplitude_j3=((5-float(i))/10.0), amplitude_j4=((5.-float(i))/20.0)) )


    def run(self):
        while(True):
            for movement in self.movements:
                iteration = 0
                while iteration < self.iterations_max:
                    if self.moving == False:
                        return
                    else:
                        mvt_percentage = float(iteration) / float(self.iterations_max)

                        go_to_next_target = movement.publish(mvt_percentage)
                        if go_to_next_target:
                            iteration += 1

    def close(self):
        self.moving = False
        for movement in self.movements:
            movement.close()

class Diamond(GenericPlugin):
    """
    Follows a diamond with FJ3 and FJ4
    """
    name = "Diamond"

    def __init__(self):
        GenericPlugin.__init__(self)

        self.fingers = ["FF","MF","RF","LF"]
        self.layout = QtGui.QHBoxLayout()
        self.frame = QtGui.QFrame()
        self.red_icon = None
        self.green_icon = None
        self.buttons = {}
        self.movements = {}
        self.moving = {}
        for finger in self.fingers:
            self.movements[finger] = Movement(finger)
            self.moving[finger] = False
            tmp_btn = QtGui.QPushButton(finger)
            self.buttons[finger] = tmp_btn
            tmp_btn.clicked.connect(partial(self.clicked, finger))
            self.layout.addWidget( tmp_btn )
        self.frame.setLayout( self.layout )
        self.window.setWidget( self.frame )

        self.dependencies = None

    def clicked(self, finger_name):
        if self.moving[finger_name]:
            self.movements[finger_name].moving = False
            self.moving[finger_name] = False
            self.movements[finger_name].join()
            self.movements[finger_name] = None
            self.buttons[finger_name].setIcon(self.green_icon)
        else:
            self.moving[finger_name] = True
            self.movements[finger_name] = Movement(finger_name)
            self.movements[finger_name].moving = True
            self.movements[finger_name].start()
            self.buttons[finger_name].setIcon(self.red_icon)

    def activate(self):
        GenericPlugin.activate(self)
        self.green_icon = QtGui.QIcon(self.parent.parent.rootPath + '/images/icons/colors/green.png')
        self.red_icon = QtGui.QIcon(self.parent.parent.rootPath + '/images/icons/colors/red.png')
        for btn in self.buttons.values():
            btn.setIcon(self.green_icon)

    def on_close(self):
        for movement in self.movements.values():
            movement.close()

        GenericPlugin.on_close(self)

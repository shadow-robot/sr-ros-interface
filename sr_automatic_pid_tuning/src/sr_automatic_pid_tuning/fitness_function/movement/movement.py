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

import roslib; roslib.load_manifest('sr_automatic_pid_tuning')
import rospy

import time, math

class BaseMovement(object):
    def __init__(self, joint_name, robot_communication, speed = 1000):
        self.joint_name = joint_name
        self.robot_communication = robot_communication

        self.sleep_time = 1.0 / speed

    def publish(self, mvt_percentage):
        self.update(mvt_percentage)
        time.sleep(self.sleep_time)

    def update(self):
        pass

class SinusoidMovement(BaseMovement):
    def __init__(self, joint_name, robot_communication,
                  speed = 1000, amplitude = 400):
        BaseMovement.__init__( self,joint_name, robot_communication,
                               speed )
        self.amplitude = amplitude
        self.value = 0.0

    def update(self, mvt_percentage):
        self.value = self.amplitude * math.sin(2.0*3.14159 * mvt_percentage/100.)
        self.robot_communication.sendupdate(self.joint_name, self.value)

    def publish(self, mvt_percentage):
        BaseMovement.publish(self, mvt_percentage)
        return self.value

class FullMovement(object):
    def __init__(self, joint_name, robot_communication ):
        self.moving = True
        self.joint_name = joint_name
        self.iterations = 100
        self.movements = [SinusoidMovement(joint_name, robot_communication)]
        self.last_target = 0.0

    def move(self):
        for movement in self.movements:
            for mvt_percentage in range(0, self.iterations):
                if self.moving == False:
                    return
                else:
                    self.last_target = movement.publish(mvt_percentage/(self.iterations/100.))

    def get_last_target(self):
        return self.last_target

class Movement(object):
    """
    Runs a full movement and record the data.
    """

    def __init__(self, robot_communication, joint_name):
        """
        """
        self.robot_communication = robot_communication
        self.joint_name = joint_name

        self.full_movement = FullMovement(joint_name, robot_communication)

        self.targets = []
        self.positions = []

    def move_and_record(self):
        #TODO: implement this
        self.full_movement.moving = True
        self.start_record()

        self.full_movement.move()

        self.stop_record()

        return self.targets,self.positions

    def record_callback(self):
        self.positions.append( self.robot_communication.get_position(self.joint_name) )
        self.targets.append( self.full_movement.get_last_target() )

    def start_record(self):
        self.robot_communication.start_record(self.joint_name, self.record_callback)

    def stop_record(self):
        self.full_movement.moving = False
        self.robot_communication.stop_record(self.joint_name)

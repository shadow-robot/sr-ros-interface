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

import roslib; roslib.load_manifest('sr_robot_lib')
import rospy

import time, math

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sr_robot_msgs.srv import ForceController
from sr_robot_msgs.msg import EthercatDebug

class EtherCAT_Hand_Lib(object):
    """
    Useful python library to communicate with the etherCAT hand.
    """
    sensors = [ "FFJ1",  "FFJ2",  "FFJ3", "FFJ4",
                "MFJ1",  "MFJ2",  "MFJ3", "MFJ4",
                "RFJ1",  "RFJ2",  "RFJ3", "RFJ4",
                "LFJ1",  "LFJ2",  "LFJ3", "LFJ4", "LFJ5",
                "THJ1",  "THJ2",  "THJ3", "THJ4", "THJ5A", "THJ5B",
                "WRJ1A", "WRJ1B", "WRJ2",
                "ACCX",  "ACCY",  "ACCZ",
                "GYRX",  "GYRY",  "GYRZ",
                "AN0",   "AN1",   "AN2",  "AN3" ]

    def __init__(self):
        """
        Useful python library to communicate with the etherCAT hand.
        """
        self.debug_subscriber = None
        self.joint_state_subscriber = None
        self.record_js_callback = None
        self.raw_values = []
        self.pid_services = {}
        self.publishers = {}
        self.positions = {}
        self.efforts = {}

        self.msg_to_send = Float64()
        self.msg_to_send.data= 0.0

        #TODO: read this from parameter server
        self.compounds = {}

        joint_to_sensor_mapping = []
        try:
            joint_to_sensor_mapping = rospy.get_param("/joint_to_sensor_mapping")
        except:
            rospy.logwarn("The parameter joint_to_sensor_mapping was not found, you won't be able to get the raw values from the the EtherCAT compound sensors.")

        for mapping in joint_to_sensor_mapping:
            if mapping[0] is 1:
                if "THJ5" in mapping[1][0]:
                    self.compounds["THJ5"] = [["THJ5A", mapping[1][1]],
                                              ["THJ5B", mapping[2][1]]]
                elif "WRJ1" in mapping[1][0]:
                    self.compounds["WRJ1"] = [["WRJ1A", mapping[1][1]],
                                              ["WRJ1B", mapping[2][1]]]

    def sendupdate(self, joint_name, target, controller_type = "effort"):
        if not self.publishers.has_key(joint_name):
            topic = "/sh_"+ joint_name.lower() +"_"+controller_type+"_controller/command"
            self.publishers[joint_name] = rospy.Publisher(topic, Float64)

        self.msg_to_send.data = math.radians( float( target ) )
        self.publishers[joint_name].publish(self.msg_to_send)

    def get_position(self, joint_name):
        """
        """
        return self.positions[joint_name]

    def get_effort(self, joint_name):
        return self.efforts[joint_name]

    def start_record(self, joint_name, callback):
        self.record_js_callback = callback

    def stop_record(self, joint_name):
        self.callback = None


    def set_pid(self, joint_name, pid_parameters):
        """
        """
        if not self.pid_services.has_key(joint_name):
            service_name =  "/realtime_loop/change_force_PID_"+joint_name
            self.pid_services[joint_name] = rospy.ServiceProxy(service_name, ForceController)

        self.pid_services[joint_name]( pid_parameters["max_pwm"],
                                       pid_parameters["sgleftref"],
                                       pid_parameters["sgrightref"],
                                       pid_parameters["f"],
                                       pid_parameters["p"],
                                       pid_parameters["i"],
                                       pid_parameters["d"],
                                       pid_parameters["imax"],
                                       pid_parameters["deadband"],
                                       pid_parameters["sign"] )

    def debug_callback(self, msg):
        self.raw_values = msg.sensors

    def joint_state_callback(self, msg):
        for name,pos,effort in zip( msg.name, msg.position, msg.effort ):
            self.positions[name] = pos
            self.efforts[name] = effort

        if self.record_js_callback is not None:
            self.record_js_callback()


    def get_raw_value(self, sensor_name):
        value = 0.0

        if self.raw_values == []:
            rospy.logwarn("No values received from the etherCAT hand.")
            return -1.0

        if sensor_name in self.compounds.keys():
            for sub_compound in self.compounds[sensor_name]:
                index = self.sensors.index( sub_compound[0] )
                value = value + ( self.raw_values[index] * sub_compound[1] )
        else:
            index = self.sensors.index( sensor_name )
            value = self.raw_values[index]

        return value

    def get_average_raw_value(self, sensor_name, number_of_samples=10):
        """
        Get the average raw value for the given sensor, average on
        number_of_samples
        """
        tmp_raw_values = []
        for i in range(0, number_of_samples):
            tmp_raw_values.append( self.get_raw_value(sensor_name) )
            time.sleep(0.002)

        average = float( sum(tmp_raw_values) )/len(tmp_raw_values)
        return average


    def activate(self):
        self.debug_subscriber = rospy.Subscriber("/debug_etherCAT_data", EthercatDebug, self.debug_callback)
        self.joint_state_subscriber = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)

    def on_close(self):
        if self.debug_subscriber is not None:
            self.debug_subscriber.unregister()
            self.debug_subscriber = None

        if self.joint_state_subscriber is not None:
            self.joint_state_subscriber.unregister()
            self.joint_state_subscriber = None


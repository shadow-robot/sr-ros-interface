#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('sr_friction_compensation')
import rospy

import time, math

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from sr_friction_compensation.lib.robot_lib import RobotLib
from sr_friction_compensation.utils.utilitarian import Utilitarian

class EtherCAT_Robot_Lib(RobotLib):
### Constructor
#
    def __init__(self, joint_name):
        self.utilitarian = Utilitarian()

        self.publisher = None
        self.subscriber = None
        self.msg_to_send = Float64()
        self.msg_to_send.data= 0.0

        self.pid_out = []
        self.position = []

        self.joint_name = joint_name
        self.index_joint = []

    def set_PID(self, P, I, D, Shift, smart_motor, hand_nb):
        #for the ethercat library, we don't want to
        # setup the PID: we're using the currently set
        # parameters
        pass

    def set_imax(self, imax_value, smart_motor, hand_nb):
        #We're not doing any imax regulation
        pass

    def set_max_temperature(self, temperature_value, smart_motor, hand_nb):
        #We're not changing the max temperature
        pass

### listvalues in python
#
    def get_current_value(self,  joint_name, hand_nb):
        #TODO: add subscriber + return last received value for position
        if self.subscriber == None:
            self.subscriber = rospy.Subscriber("/joint_states", JointState, self.callback_)

        iterations = 0
        while len(self.position) == 0:
            if iterations > 1000:
                print "Couldn't read the current position for ", joint_name
                return
            time.sleep(0.001)

        #return the last position
        return self.position[-1]

    def callback_(self, msg):
        #pid_out = effort because we're using an internal force control loop
        pos = 0.0
        effort = 0.0
        names = []
        if "J0" in self.joint_name:
            finger = self.joint_name.split("J0")
            names.append( finger[0]+"J1" )
            names.append( finger[0]+"J2" )
        else:
            names.append( self.joint_name )

        if self.index_joint == []:
            for index,name in enumerate(msg.name):
                for joint_name in names:
                    if name in joint_name:
                        self.index_joint.append( index )

        if len( self.index_joint ) == 0:
            rospy.logerr("Joint not found: " + self.joint_name)
            return

        for index_joint in self.index_joint:
            pos += msg.position[index_joint]
            effort += msg.effort[index_joint]

        self.position.append( pos )
        self.pid_out.append( effort )

    def start_record(self, joint_name, hand_nb):
        #TODO: start filling a vector with the positions and the output of the controller
        # save in self.position_hex and self.pid_out_hex (as floats)
        if self.subscriber == None:
            self.subscriber = rospy.Subscriber("/joint_states", JointState, self.callback_)

        return [None , None]

    def stop_record(self, process):
        #TODO: stop recording
        if self.subscriber != None:
            self.subscriber.unregister()
            self.subscriber = None

    def get_data(self, data_location):
        # here data location is the name of the text file
        # Put data into lists

        return [self.position, self.pid_out]

### Sendupdate in python
#
    def sendupdate(self, joint_name, hand_nb, value):
        #TODO: send a target in velocity
        if self.publisher == None:
            topic = "/sh_"+ joint_name.lower() +"_velocity_controller/command"
            self.publisher = rospy.Publisher(topic, Float64)

        self.msg_to_send.data = math.radians( float( value ) )
        self.publisher.publish(self.msg_to_send)

### Contrlr in python
#
    def contrlr(self, smart_motor, options, hand_nb ):
        #configures the motor, not needed here
        pass

### Send the U_map table to the firmware
#
    def send_u_map_to_firmware(self, u_map_position, u_map_pid_out, node_id, direction, joint_name, hand_nb):
        # send the u_map table
        date = time.localtime()
        output_file =  "/tmp/" + str(date.tm_year)+ '_' + str(date.tm_mon)+ '_' +str(date.tm_mday)+ '_' +str(date.tm_hour)+ '_' +str(date.tm_min)+ '_' +str(date.tm_sec)+'/'+ joint_name + "_" + direction +"_friction_compensation.txt"

        #TODO: write the u map somewhere. (ask the user for it?)
        # Generate u_map file
        # u_map_position -> u_map_pid_out


#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('sr_friction_compensation')
import rospy

import subprocess , time, string

from sr_friction_compensation.lib.robot_lib import RobotLib
from sr_friction_compensation.utils.utilitarian import Utilitarian

class EtherCAT_Robot_Lib(RobotLib):
### Constructor
#
    def __init__(self):
        self.utilitarian = Utilitarian()

        self.pid_out = []
        self.position = []

    def set_PID(self, P, I, D, Shift, smart_motor):
        #for the ethercat library, we don't want to
        # setup the PID: we're using the currently set
        # parameters
        pass

    def set_imax(self, imax_value, smart_motor):
        #We're not doing any imax regulation
        pass

    def set_max_temperature(self, temperature_value, smart_motor):
        #We're not changing the max temperature
        pass

### listvalues in python
#
    def get_current_value(self,  joint_name, hand_nb):
        #TODO: add subscriber + return last received value for position


    def start_record(self):
        #TODO: start filling a vector with the positions and the output of the controller
        # save in self.position_hex and self.pid_out_hex (as floats)
        return [None , None]

    def stop_record(self, process):
        #TODO: stop recording
        pass

    def get_data(self, data_location):
        # here data location is the name of the text file
        # Put data into lists
        return [self.position, self.pid_out]

### Sendupdate in python
#
    def sendupdate(self, joint_name, hand_nb, value):
        #TODO: send a target in velocity
        pass

### Contrlr in python
#
    def contrlr(self, smart_motor, options ):
        #configures the motor, not needed here
        pass

### Send the U_map table to the firmware
#
    def send_u_map_to_firmware(self, u_map_position, u_map_pid_out, node_id, direction, joint_name):
        # send the u_map table
        date = time.localtime()
        output_file =  "/tmp/" + str(date.tm_year)+ '_' + str(date.tm_mon)+ '_' +str(date.tm_mday)+ '_' +str(date.tm_hour)+ '_' +str(date.tm_min)+ '_' +str(date.tm_sec)+'/'+ joint_name + "_" + direction +"_friction_compensation.txt"

        #TODO: write the u map somewhere. (ask the user for it?)
        # Generate u_map file
        # u_map_position -> u_map_pid_out


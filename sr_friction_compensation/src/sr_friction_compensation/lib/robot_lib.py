#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('sr_friction_compensation')
import rospy

class RobotLib(object):
    def __init__(self):
        pass

    def set_PID(self, P, I, D, Shift, joint_name, hand_nb):
        pass

    def set_imax(self, imax_value, joint_name, hand_nb):
        pass

    def set_max_temperature(self, temperature_value, joint_name, hand_nb):
        pass

    def get_current_value(self, joint_name, hand_nb):
        pass

    def start_record(self, joint_name, hand_nb):
        pass

    def stop_record(self, process):
        pass

    def get_data(self, data_location):
        pass

    def sendupdate(self, joint_name, hand_nb, value):
        pass

    def send_u_map_to_firmware(self, u_map_position, u_map_pid_out, direction, joint_name, hand_nb):
        pass


#!/usr/bin/env python
# -*- coding: utf-8 -*-

import roslib; roslib.load_manifest('sr_friction_compensation')
import rospy

import time, string
from sr_friction_compensation.inputs.u_map_data_acquisition import U_Map_Data_Acquisition
from sr_friction_compensation.lib.python_robot_lib import Python_Robot_Lib

class U_Map_Data_Acquisition_Python(U_Map_Data_Acquisition):
    ### Constructor
    #
    def __init__(self, joint_name, direction, imax_regulation):
        U_Map_Data_Acquisition.__init__(self, joint_name, direction)

        self.imax_regulation = imax_regulation
        self.lib = Python_Robot_Lib()
        self.position_dead_band = 0.5
        self.initial_imax = []
        self.imax_inc = []
        self.imax_dec = []
        self.imax_time =[]
        self.imax_min_delta_p = []
        self.imax_max_delta_p = []
        # CAUTION: node_id is wrong as no hand number is given (it is useless here)
        [self.min_angle, self.max_angle, self.node_id, self.motor, self.min_osc_angle, self.max_osc_angle] = self.utilitarian.joint_characteristics(self.joint_name, 'hand_nb')
        self.position_sensor = self.joint_name + "_Pos"
        self.target = self.joint_name +"_Target"
        self.motor_debug = "smart_motor_debug_" + self.motor
        self.smart_motor = "smart_motor_" + self.motor +".0"

    ### Run measurements in Python
    #
    def measurement_process(self, P, I, D, shift):
        ## Measurements preparation
        #
        # Configure the motor
        self.lib.set_PID(P, I, D, Shift, self.smart_motor)

        # Set motion parameter
        self.set_motion_parameters()

        # Drive the joint to the initial position
        self.drive_to_initial_position(self.position_dead_band)

        # Measurements
        #
        # Start recording data
        [process, data_location] = self.lib.start_record()

        # Move the joint
        self.move_joint_and_regule_imax( self.position_dead_band)

        # Stop recording data
        self.lib.stop_record(process)

        # Return the measurement file
        return data_location

    ### Drive the joint to initial position
    #
    def drive_to_initial_position(self, position_dead_band):
        if imax_regulation:
            # Set the imax value high enough to move faster
            self.lib.set_imax(3000, self.smart_motor)

        cur_pos = self.lib.get_current_value(self.position_sensor)
        # Move until it reaches the initial position
        while (abs(cur_pos-self.initial_position)> position_dead_band):
            self.lib.sendupdate(self.position_sensor, self.initial_range_end)
            cur_pos = self.lib.get_current_value(self.position_sensor)
            time.sleep(0.5)

        if imax_regulation:
          # Set the imax and max temperature to 0, to make sure that the joint stops
          self.lib.set_imax(0, self.smart_motor)
          self.lib.set_max_temperature(0, self.smart_motor)

    ### Carry out Imax regulation
    #
    def move_joint_and_regule_imax(self, position_dead_band):

        if imax_regulation:
            # Set imax and maximum temperature values to enable motion
            self.set_imax_regulation_parameters()
            imax = self.initial_imax
            self.lib.set_imax(imax, self.smart_motor)
            self.lib.set_max_temperature(15000, self.smart_motor)

        # Check if the joint has reached the final position
        cur_pos = self.lib.get_current_value(self.position_sensor)
        while abs(cur_pos - self.final_position) > position_dead_band:
            if self.stopped:
                break

            # send the target position (repeated at each step maybe not always useful)
            self.lib.sendupdate(self, self.position_sensor, self.final_range_end)
            time.sleep(self.imax_time)

            # Regulate the I_max value according to the motion speed
            prev_pos = cur_pos
            cur_pos = self.lib.get_current_value(self.position_sensor)

            if self.imax_regulation:
                # if the joint is too slow it increases the imax value (otherwise sometimes the joint is stuck)
                if abs(cur_pos-prev_pos) < self.imax_min_delta_p:
                    imax += self.imax_inc
                    self.lib.set_imax(imax, self.smart_motor)

                # if the joint is too fast, it decreases the imax value
                elif abs(cur_pos-prev_pos) > self.imax_max_delta_p:
                    imax -= self.imax_dec
                    self.lib.set_imax(imax, self.smart_motor)

        # Set the imax and max temperature to 0, to make sure that the joint stops
        self.lib.set_imax(0, self.smart_motor)
        self.lib.set_max_temperature(0, self.smart_motor)


    ### set imax regulation parameters
    #
    def set_imax_regulation_parameters(self):
        if self.direction == 'forward':
            self.initial_imax = 1616
            self.imax_inc = 5
            self.imax_dec = 10
            self.imax_time = 1
            self.imax_min_delta_p = 0.55
            self.imax_max_delta_p = 0.78
        else:
            self.initial_imax = 600
            self.imax_inc = 20
            self.imax_dec = 10
            self.imax_time = 2
            self.imax_min_delta_p = 0.55
            self.imax_max_delta_p = 1

    ### Set motion parameters
    #
    def set_motion_parameters(self):
        if self.direction == 'forward':
            self.initial_position = self.min_osc_angle
            self.initial_range_end = self.min_angle
            self.final_position = self.max_osc_angle
            self.final_range_end = self.max_angle
        else:
            self.initial_position = self.max_osc_angle
            self.initial_range_end = self.max_angle
            self.final_position = self.min_osc_angle
            self.final_range_end = self.min_angle



    ### Prepare and run data treatment
    #
    def run_data_acquisition(self, P, I, D, shift):
        # Run measurements
        #[data_location] = self.measurement_process(P, I, D, shift)
        # note: As the speed is very low:
        # D value should be closed to 0
        # I value should be rather high

        # For tests ONLY
        data_location = ""
        if self.direction == 'forward':
            data_location = "/tmp/test.txt"
        else:
            data_location = "/tmp/ffj4_pos_pid_backward_unique.txt"

        print "data loc: ", data_location

        # get measured data
        [self.position_hex, self.pid_out_hex] = self.lib.get_data( data_location)

        print "ok got data"


        # run data_treatment
        [self.position, self.pid_out] = self.data_treatment(self.position_hex, self.pid_out_hex)

        print "ran treatment: ", self.position, " ", self.pid_out

        # return the final data
        return [self.position, self.pid_out]

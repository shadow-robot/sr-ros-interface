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
    def __init__(self, joint_name, hand_nb, direction, lib):
        U_Map_Data_Acquisition.__init__(self, joint_name, hand_nb, direction)

        self.lib = lib
        self.position_dead_band = 1
        [self.min_angle, self.max_angle, self.min_osc_angle, self.max_osc_angle] = self.utilitarian.joint_characteristics(self.joint_name, self.hand_nb)
        self.initial_position = []
        self.initial_range_end = []
        self.final_position = []
        self.final_range_end = []
    ### Run measurements in Python
    #
    def measurement_process(self, P, I, D, shift):
        ## Measurements preparation
        #
        # Configure the motor
        print "set pid"
        self.lib.set_PID(P, I, D, shift, self.joint_name, self.hand_nb)

        # Set motion parameter
        print "set motion param"
        self.set_motion_parameters()

        print "drive to initial pos"
        # Drive the joint to the initial position
        self.drive_to_initial_position(self.position_dead_band)

        print "starting to record"
        ## Measurements
        ##
        ## Start recording data
        [process, data_location] = self.lib.start_record(self.joint_name, self.hand_nb)
        print 'started'
        time.sleep(1)

        ## Move the joint
        self.move_joint( self.position_dead_band)

        ## Stop recording data
        self.lib.stop_record(process)
        print 'stop'

        ## Return the measurement file
        print data_location
        return data_location


    ### Drive the joint to initial position
    #
    def drive_to_initial_position(self, position_dead_band):
        print "driving to: ", self.initial_position, " deadband = ", position_dead_band
        cur_pos = self.lib.get_current_value(self.joint_name, self.hand_nb)
        initial_time = time.time()
        t = time.time()
        #print cur_pos
        # Move until it reaches the initial position
        print "pos: ",cur_pos, " target: ", self.initial_position, "error: ", abs(cur_pos - self.initial_position), " deadband: ",position_dead_band
        while (abs(cur_pos-self.initial_position)> position_dead_band)and (abs(initial_time-t)<100):
            print "pos: ",cur_pos, " target: ", self.initial_position, "error: ", abs(cur_pos - self.initial_position), " deadband: ",position_dead_band
            #print abs(cur_pos-self.initial_position)
            self.lib.sendupdate(self.joint_name, self.hand_nb, self.initial_position)
            #print self.initial_range_end
            cur_pos = self.lib.get_current_value(self.joint_name, self.hand_nb)
            #print cur_pos
            time.sleep(0.5)
            t = time.time()
            #print t



        self.lib.set_max_temperature(0, self.joint_name, self.hand_nb)

    ### Move the joint
    #
    def move_joint(self, position_dead_band):

        self.lib.set_max_temperature(15000, self.joint_name, self.hand_nb)

        # Check if the joint has reached the final position
        cur_pos = self.lib.get_current_value(self.joint_name, self.hand_nb)
        print cur_pos
        initial_time = time.time()
        t = time.time()
        while (abs(cur_pos - self.final_position) > position_dead_band) and (abs(initial_time-t)<100):
            print abs(cur_pos - self.final_position)
            #if self.stopped:
                #break

            # send the target position (repeated at each step maybe not always useful)
            self.lib.sendupdate( self.joint_name, self.hand_nb, self.final_range_end)
            #time.sleep(self.imax_time)
            time.sleep(0.5)

            # Regulate the I_max value according to the motion speed
            prev_pos = cur_pos
            cur_pos = self.lib.get_current_value(self.joint_name, self.hand_nb)


            t = time.time()

        # Set the imax and max temperature to 0, to make sure that the joint stops
        self.lib.set_max_temperature(0, self.joint_name, self.hand_nb)



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
        data_location = self.measurement_process(P, I, D, shift)
        # note: As the speed is very low:
        # D value should be closed to 0
        # I value should be rather high

        ## For tests ONLY
        #data_location = ""
        #if self.direction == 'forward':
            ##data_location = "/tmp/test.txt"
            #data_location = "test.txt"
        #else:
            ##data_location = "/tmp/ffj4_pos_pid_backward_unique.txt"
            #data_location = "ffj4_pos_pid_backward_unique.txt"

        #print "data loc: ", data_location

        # get measured data
        [self.position_float, self.pid_out_float] = self.lib.get_data( data_location)

        print "ok got data"

        # run data_treatment
        [self.position, self.pid_out] = self.data_treatment(self.position_float, self.pid_out_float)

        #print "ran treatment: ", self.position, " ", self.pid_out

        # return the final data
        return [self.position, self.pid_out]

# ONLY for tests
if __name__ == "__main__":
    #print 'tets'
    lib = Python_Robot_Lib()
    joint_name = 'THJ1'
    hand_nb = '49'
    #direction = 'forward'
    direction = 'backward'
    data_acquisition = U_Map_Data_Acquisition_Python( joint_name , hand_nb , direction , lib)
    data_acquisition.measurement_process(P = 0, I= -498, D=0, shift=7)


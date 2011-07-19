#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys, time, string
sys.path.append('../')
from u_map_data_acquisition import U_Map_Data_Acquisition
from lib.python_robot_lib import Python_Robot_Lib

class U_Map_Data_Acquisition_Python(U_Map_Data_Acquisition):
  
### Constructor
#                
    def __init__(self, joint_name, direction):
        U_Map_Data_Acquisition.__init__(self, joint_name, direction)
        
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
        options = 'sensor ' + self.position_sensor + ' target ' + self.target
        self.lib.contrlr(self.smart_motor, options )
        
        options = 'p ' + P + ' i ' + I + ' d ' + D + ' motor_maxforce 16384 motor_safeforce 16383 force_out_shift 255 sensor_out_shift ' + Shift + ' sensor_deadband 128 sensor_offset 0 max_temperature 15000  max_current 200 type_of_sensor 0 type_of_setpoint 0'
        self.lib.contrlr(self.smart_motor, options )
                   
      
      # Drive the joint to the initial position
        self.drive_to_initial_position(self.position_dead_band)
      
    # Measurements
    #
      # Start recording data  
        options = '-d 10 -r -p -l'
        sensors = self.motor_debug + '.13' + self.motor_debug + '.3'
        # Each output file is recorded in a dated folder
        date = time.localtime()
        output_file = str(date.tm_year)+ '_' + str(date.tm_mon)+ '_' +str(date.tm_mday)+ '_' +str(date.tm_hour)+ '_' +str(date.tm_min)+ '_' +str(date.tm_sec)+'/measurement_file.txt'
        process = self.lib.start_record(options, sensors, output_file)          
      
      # Move the joint
        self.move_joint_and_regule_imax( self.position_dead_band)        
      
      # Stop recording data  
        self.lib.stop_record(process)
      
      # Return the measurement file
        return output_file
        
### Drive the joint to initial position 
#        
    def drive_to_initial_position(self, position_dead_band):
      # Set the imax value high enough to move faster 
        options = 'sensor_imax 3000'
        self.lib.contrlr(self.smart_motor, options)        
        cur_pos = self.lib.get_current_value(self.position_sensor)
      # Move until it reaches the initial position
        while (abs(cur_pos-self.initial_position)> position_dead_band):                        
            self.lib.sendupdate(self.position_sensor, self.initial_range_end)
            cur_pos = self.lib.get_current_value(self.position_sensor)
            time.sleep(0.5)
      # Set the imax and max temperature to 0, to make sure that the joint stops
        options = 'sensor_imax 0 max_temperature 0'
        self.lib.contrlr(self.smart_motor, options)        
                    
### Carry out Imax regulation
#
    def move_joint_and_regule_imax(self, position_dead_band):
      # Set imax and maximum temperature values to enable motion
        self.set_motion_and_imax_regulation_parameters()
        imax = self.initial_imax
        options = 'sensor_imax ' + imax + ' max_temperature 15000'
        self.lib.contrlr(self.smart_motor, options)
        
      # Check if the joint has reached the final position
        cur_pos = self.lib.get_current_value(self.position_sensor)
        while abs(cur_pos - self.final_position) > position_dead_band:
          
          # send the target position (repeated at each step maybe not always useful)
            self.lib.sendupdate(self, self.position_sensor, self.final_range_end)
            time.sleep(self.imax_time)
            
          # Regulate the I_max value according to the motion speed  
            prev_pos = cur_pos
            cur_pos = self.lib.get_current_value(self.position_sensor)
            
            # if the joint is too slow it increases the imax value (otherwise sometimes the joint is stuck)
            if abs(cur_pos-prev_pos) < self.imax_min_delta_p:
                imax += self.imax_inc                  
                options = 'sensor_imax '+ imax
                self.lib.contrlr(self.smart_motor, options)        
            
            # if the joint is too fast, it decreases the imax value 
            elif abs(cur_pos-prev_pos) > self.imax_max_delta_p:
                imax -= self.imax_dec
                options = 'sensor_imax '+ imax
                self.lib.contrlr(self.smart_motor, options)        
      
      # Set the imax and max temperature to 0, to make sure that the joint stops
        options = 'sensor_imax 0 max_temperature 0'
        self.lib.contrlr(self.smart_motor, options)        
                                
        
### set motion and imax regulation parameters
#
    def set_motion_and_imax_regulation_parameters(self):
        if self.direction == 'forward':
            self.initial_position = self.min_osc_angle
            self.initial_range_end = self.min_angle
            self.final_position = self.max_osc_angle
            self.final_range_end = self.max_angle
            self.initial_imax = 1616
            self.imax_inc = 5
            self.imax_dec = 10
            self.imax_time = 1
            self.imax_min_delta_p = 0.55
            self.imax_max_delta_p = 0.78
        else:
            self.initial_position = self.max_osc_angle
            self.initial_range_end = self.max_angle
            self.final_position = self.min_osc_angle
            self.final_range_end = self.min_angle
            self.initial_imax = 600
            self.imax_inc = 20
            self.imax_dec = 10
            self.imax_time = 2
            self.imax_min_delta_p = 0.55  
            self.imax_max_delta_p = 1
        

### Read the measured data from the text files and save them into vectors
#
      # Input:
          # Text file name
      # Actions:
          # Read the measured data from the text file and save them into vectors
          # Text file contains 3 columns:
                    # line number
                    # position values
                    # pid_out values
    def text_file_to_vectors(self, text_file_name):
      
      # Copy the data from text files into lists
      #            
            file_open=open(text_file_name, 'r')
            frictions_data=file_open.readlines()
            
          # The number of lines in the text file     
            line_number = len(frictions_data)
          
          # Temporary lists declaration
            measured_position_hexa=[]
            measured_pid_out_hexa=[]
          
          # Copy list into the lists
            for i in range(line_number):
              # split each line (data in files are seperated by spaces)
                words = string.split(frictions_data[i], ' ') 
              # make sure that each line contains 3 elements
                if len(words) >= 3:
                  # Copy data into lists
                    measured_position_hexa = measured_position_hexa + [words[1]]
                    measured_pid_out_hexa = measured_pid_out_hexa + [words[2]]                                 
             
            return [measured_position_hexa, measured_pid_out_hexa]
        
            #print measured_position_hexa
            #print measured_pid_out_hexa



### Prepare and run data treatment
#
    def run_data_acquisition(self, P, I, D, shift):
      # Run measurements
          #[measurement_file] = self.measurement_process(P, I, D, shift)
            # note: As the speed is very low:
                        # D value should be closed to 0
                        # I value should be rather high
      # For tests ONLY
          if self.direction == 'forward':
              measurement_file = "test.txt"
          else:
              measurement_file = "ffj4_pos_pid_backward_unique.txt"

      # Put data into lists          
          [self.position_hex, self.pid_out_hex] = self.text_file_to_vectors(measurement_file)     
      
      # run data_treatment          
          [self.position, self.pid_out] = self.data_treatment(self.position_hex, self.pid_out_hex)  
      
      # return the final data
          return [self.position, self.pid_out]


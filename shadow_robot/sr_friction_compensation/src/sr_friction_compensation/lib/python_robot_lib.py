#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('sr_friction_compensation')
import rospy

import subprocess , time, string

from sr_friction_compensation.utils.utilitarian import Utilitarian

class Python_Robot_Lib(object):
  
### Constructor
#
    def __init__(self):
        self.utilitarian = Utilitarian()

    def set_PID(self, P, I, D, Shift, smart_motor):
        options = 'sensor ' + self.position_sensor + ' target ' + self.target
        self.contrlr(smart_motor, options )
        
        # Set the PID values
        options = 'p ' + P + ' i ' + I + ' d ' + D + ' motor_maxforce 16384 motor_safeforce 16383 force_out_shift 255 sensor_out_shift ' + Shift + ' sensor_deadband 128 sensor_offset 0 max_temperature 15000  max_current 200 type_of_sensor 0 type_of_setpoint 0'
        self.contrlr(smart_motor, options )
        
        # Set the imax value
        options = 'sensor_imax 3000'
        self.contrlr(smart_motor, options)
            
    def set_imax(self, imax_value, smart_motor):
        options = 'sensor_imax '+imax_value
        self.contrlr(smart_motor, options)
        
    def set_max_temperature(self, temperature_value, smart_motor):
        options = 'max_temperature '+temperature_value
        self.lib.contrlr(smart_motor, options)



### listvalues in python
#              
    def get_current_value(self, sensor):        
        command = "listvalues -i 1 -d 100 " + sensor        
        answer = self.utilitarian.run_command(command)
        #p = subprocess.Popen(command.split(),stdout=subprocess.PIPE)
        #p.wait()
        #answer = p.stdout.read()
        result = float(answer.strip('\n'))
        return result        
        #print 'result = ' + result.strip('\n')           
    
    def start_record(self):        
        options = '-d 10 -r -p -l'
        sensors = self.motor_debug + '.13' + self.motor_debug + '.3'
        # Each output file is recorded in a dated folder
        date = time.localtime()
        output_file = str(date.tm_year)+ '_' + str(date.tm_mon)+ '_' +str(date.tm_mday)+ '_' +str(date.tm_hour)+ '_' +str(date.tm_min)+ '_' +str(date.tm_sec)+'/measurement_file.txt'
        
        command = 'listvalues '+ options + ' ' + sensors + ' -o ' + output_file
        p = subprocess.Popen(command.split(),stdout=subprocess.PIPE)
        return [p , output_file]
        
    def stop_record(self, process):
        process.terminate()    

    def get_data(self, data_location):
        # here data location is the name of the text file
        # Put data into lists
        [position_hex, pid_out_hex] = self.utilitarian.text_file_to_vectors(data_location)
        return [position_hex, pid_out_hex]

### Sendupdate in python
#
    def sendupdate(self, sensor, value):
        command = 'sendupdate ' + sensor + ' ' + value
        answer = self.utilitarian.run_command(command)    

### Contrlr in python
#
    def contrlr(self, smart_motor, options ):
        command = 'contrlr ' + smart_motor + ' ' + options
        answer = self.utilitarian.run_command(command)               
        
### Send the U_map table to the firmware
#
    def send_u_map_to_firmware(self, u_map_position, u_map_pid_out, node_id, direction):     
        # Generate u_map file
        self.utilitarian.write_u_map_in_text_file(u_map_position, u_map_pid_out, node_id, direction)
        
        # send the u_map table
        date = time.localtime()        
        output_file =  str(date.tm_year)+ '_' + str(date.tm_mon)+ '_' +str(date.tm_mday)+ '_' +str(date.tm_hour)+ '_' +str(date.tm_min)+ '_' +str(date.tm_sec)+'/'+ self.joint_name + "_" + self.direction +"_friction_compensation.txt"
        
        command = "sendcal "+ output_file   
        answer = self.utilitarian.run_command(command)

    
if __name__ == "__main__":
    prl = Python_Robot_Lib()
    position_sensor = 'FFJ4_Pos'
    
    #cur_pos = prl.get_current_value(position_sensor)
    #prl.get_current_value(position_sensor)
    #print cur_pos
    
    options = '-d 100'
    output_file = 'toto.txt'
    
    process = prl.start_record(options, position_sensor, output_file)
    time.sleep(1)
    prl.stop_record(process)
    print 'terminate'
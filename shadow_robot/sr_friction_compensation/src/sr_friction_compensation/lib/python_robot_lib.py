#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('sr_friction_compensation')
import rospy

import subprocess , time, string

from sr_friction_compensation.lib.robot_lib import RobotLib
from sr_friction_compensation.utils.utilitarian import Utilitarian

class Python_Robot_Lib(RobotLib):

    ### Constructor
    #
    def __init__(self):
        self.utilitarian = Utilitarian()

    ### Configure the motor
    #
    def set_PID(self, P, I, D, Shift, joint_name, hand_nb):
        [node_id, position_sensor, target, motor_debug, smart_motor] = self.get_joint_ids( joint_name, hand_nb)
        options = 'sensor ' + position_sensor + ' target ' + target
        self.contrlr(smart_motor, options )

        # Set the PID values
        options = 'p ' + P + ' i ' + I + ' d ' + D + ' motor_maxforce 16384 motor_safeforce 16383 force_out_shift 255 sensor_out_shift ' + Shift + ' sensor_deadband 128 sensor_offset 0 max_temperature 15000  max_current 200 type_of_sensor 0 type_of_setpoint 0'
        self.contrlr(smart_motor, options )

        # Set the imax value
        options = 'sensor_imax 3000'
        self.contrlr(smart_motor, options)

    ### Set imax value
    #
    def set_imax(self, imax_value, joint_name, hand_nb):
        [node_id, position_sensor, target, motor_debug, smart_motor] = self.get_joint_ids( joint_name, hand_nb)
        options = 'sensor_imax '+imax_value
        self.contrlr(smart_motor, options)

    ### Set the maximum temperature
    #
    def set_max_temperature(self, temperature_value, joint_name, hand_nb):
        [node_id, position_sensor, target, motor_debug, smart_motor] = self.get_joint_ids( joint_name, hand_nb)
        options = 'max_temperature '+temperature_value
        self.lib.contrlr(smart_motor, options)



    ### Return the position value
    #
    def get_current_value(self, joint_name, hand_nb):
        [node_id, position_sensor, target, motor_debug, smart_motor] = self.get_joint_ids( joint_name, hand_nb)
        command = "listvalues -i 1 -d 100 " + position_sensor
        answer = self.utilitarian.run_command(command)
        #p = subprocess.Popen(command.split(),stdout=subprocess.PIPE)
        #p.wait()
        #answer = p.stdout.read()
        result = float(answer.strip('\n'))
        return result
        #print 'result = ' + result.strip('\n')

    ### Launch the recording of position and pid output in a file
    #
    def start_record(self, joint_name, hand_nb):
        [node_id, position_sensor, target, motor_debug, smart_motor] = self.get_joint_ids( joint_name, hand_nb)
        options = '-d 10 -r -p -l'
        sensors = motor_debug + '.13' + motor_debug + '.3'
        # Each output file is recorded in a dated folder
        date = time.localtime()
        output_file = str(date.tm_year)+ '_' + str(date.tm_mon)+ '_' +str(date.tm_mday)+ '_' +str(date.tm_hour)+ '_' +str(date.tm_min)+ '_' +str(date.tm_sec)+'/measurement_file.txt'

        command = 'listvalues '+ options + ' ' + sensors + ' -o ' + output_file
        p = subprocess.Popen(command.split(),stdout=subprocess.PIPE)
        return [p , output_file]

    ### Stop the record of position and PID output
    #
    def stop_record(self, process):
        process.terminate()

    ### Read measured data in file and return them in float format
    #
    def get_data(self, data_location):
        # here data location is the name of the text file
        # Put data into lists
        [position_hex, pid_out_hex] = self.utilitarian.text_file_to_vectors(data_location)
        # Convert from hex to dec
        # position
        float_position = self.utilitarian.position_conversion_to_float(position_hex)

        # pid_out
        float_pid_out = self.utilitarian.pid_out_conversion_to_float(pid_out_hex)

        return [float_position, float_position]


    ### Sendupdate in python (send a position target)
    #
    def sendupdate(self, joint_name, hand_nb, value):
        [node_id, position_sensor, target, motor_debug, smart_motor] = self.get_joint_ids( joint_name, hand_nb)
        command = 'sendupdate ' + position_sensor + ' ' + value
        answer = self.utilitarian.run_command(command)

    ### Contrlr in python
    #
    def contrlr(self, joint_name, hand_nb, options ):
        [node_id, position_sensor, target, motor_debug, smart_motor] = self.get_joint_ids( joint_name, hand_nb)
        command = 'contrlr ' + smart_motor + ' ' + options
        answer = self.utilitarian.run_command(command)

    ### Send the U_map table to the firmware
    #
    def send_u_map_to_firmware(self, u_map_position, u_map_pid_out, direction, joint_name, hand_nb):
        [node_id, position_sensor, target, motor_debug, smart_motor] = self.get_joint_ids( joint_name, hand_nb)

        # Conversion to the U_map format
        final_u_map_position = self.utilitarian.set_pos_to_u_map_format(u_map_position)
        final_u_map_pid_out = self.utilitarian.set_pid_to_u_map_format(u_map_pid_out)

        # send the u_map table
        date = time.localtime()
        output_file =  "/tmp/" + str(date.tm_year)+ '_' + str(date.tm_mon)+ '_' +str(date.tm_mday)+ '_' +str(date.tm_hour)+ '_' +str(date.tm_min)+ '_' +str(date.tm_sec)+'/'+ joint_name + "_" + direction +"_friction_compensation.txt"

        # Generate u_map file
        self.utilitarian.write_u_map_in_text_file(final_u_map_position, final_u_map_pid_out, output_file, node_id, direction )

        command = "sendcal "+ output_file
        #answer = self.utilitarian.run_command(command)

    ### Return firmware ids of the joint
    #
    def get_joint_ids(self, joint_name, hand_nb):
        # First finger
        if (joint_name == "FFJ1"):
            motor = 'ff0';
            node_id = 'node ' + hand_nb + '12' + '0310'
        elif (joint_name == "FFJ2" ):
            motor = 'ff0';
            node_id = 'node ' + hand_nb + '12' + '0310'
        elif (joint_name == "FFJ3"):
            motor = 'ff3';
            node_id = 'node ' + hand_nb + '13'+'0310'
        elif (joint_name == "FFJ4"):
            motor = 'ff4';
            node_id = 'node ' + hand_nb + '11' + '0310'

        # Medium finger
        elif ( joint_name == 'MFJ1'):
            motor = 'mf0';
            node_id = 'node ' + hand_nb + '02' + '0310'
        elif ( joint_name == 'MFJ2'):
            motor = 'mf0';
            node_id = 'node ' + hand_nb + '02' + '0310'
        elif ( joint_name == 'MFJ3'):
            motor = 'mf3';
            node_id = 'node ' + hand_nb + '16' + '0310'
        elif ( joint_name == 'MFJ4'):
            motor = 'mf4';
            node_id = 'node ' + hand_nb + '01' + '0310'

        # Right Finger:
        elif ( joint_name == 'RFJ1'):
            motor = 'rf0';
            node_id = 'node ' + hand_nb + '04' + '0310'
        elif ( joint_name == 'RFJ2'):
            motor = 'rf0';
            node_id = 'node ' + hand_nb + '04' + '0310'
        elif ( joint_name == 'RFJ3'):
            motor = 'rf3';
            node_id = 'node ' + hand_nb + '05' + '0310'
        elif ( joint_name == 'RFJ4'):
            motor = 'rf4';
            node_id = 'node ' + hand_nb + '03' + '0310'

        # Little finger
        elif ( joint_name == 'LFJ1'):
            motor = 'lf0';
            node_id = 'node ' + hand_nb + '09' + '0310'
        elif ( joint_name == 'LFJ2'):
            motor = 'lf0';
            node_id = 'node ' + hand_nb + '09' + '0310'
        elif ( joint_name == 'LFJ3'):
            motor = 'lf3';
            node_id = 'node ' + hand_nb + '08' + '0310'
        elif ( joint_name == 'LFJ4'):
            motor = 'lf4';
            node_id = 'node ' + hand_nb + '10' + '0310'
        elif ( joint_name == 'LFJ5'):
            motor = 'lf5';
            node_id = 'node ' + hand_nb + '06' + '0310'

        # Thumb
        elif ( joint_name == 'THJ1'):
            motor = 'th1';
            node_id = 'node ' + hand_nb + '14' + '0310'
        elif ( joint_name == 'THJ2'):
            motor = 'th2';
            node_id = 'node ' + hand_nb + '15' + '0310'
        elif ( joint_name == 'THJ3'):
            motor = 'th3';
            node_id = 'node ' + hand_nb + '07' + '0310'
        elif ( joint_name == 'THJ4'):
            motor = 'th4';
            node_id = 'node ' + hand_nb + '19' + '0310'
        elif ( joint_name == 'THJ5'):
            motor = 'th5';
            node_id = 'node ' + hand_nb + '20' + '0310'

        # Wrist
        elif ( joint_name == 'WRJ1'):
            motor = 'wr1';
            node_id = 'node ' + hand_nb + '18' + '0310'
        elif ( joint_name == 'WRJ2'):
            motor = 'wr2';
            node_id = 'node ' + hand_nb + '17' + '0310'
        else:
          print "The joint is wrong it should be in the following format 'FFJ4'"
          return
        position_sensor = joint_name + "_Pos"
        target = joint_name +"_Target"
        motor_debug = "smart_motor_debug_" + motor
        smart_motor = "smart_motor_" + motor +".0"

        return [node_id, position_sensor, target, motor_debug, smart_motor]



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

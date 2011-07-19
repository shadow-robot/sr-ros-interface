#!/usr/bin/env python
# -*- coding: utf-8 -*-
import subprocess
class Utilitarian(object):
  
### Constructor
#
    def __init__(self):
      # Constant for conversion from hex to decimals
	self.sign_parameter = int("0x7fff", 0)
	self.twos_complement_shift = int("0x10000", 0)

### Delete tuples (positions, pid_out) recorded several times
#
    def delete_repeated_tuples(self, position, pid_out):        
        i = 0
        final_position = position
        final_pid_out = pid_out                
        while i < min(len(final_position), len(final_pid_out))-1:                        
            pos_tested = final_position[i]
            pid_out_tested = final_pid_out[i]
            j = i+1
            while j < min(len(final_position), len(final_pid_out)): 
              if (final_position[i]==final_position[j]) and (final_pid_out[i] == final_pid_out[j]):
                  final_position.pop(j)
                  final_pid_out.pop(j)
              else:
                  j += 1
            i += 1
        return [final_position, final_pid_out]

### Conversion of the position from hexadecimal values to decimal values
#  
      # hexadecimal format:
	# signed floats (twos complement) 16 bits; fraction length 8 bits
	# positive values: from 0x0000 to 0x7fff
	# negative values: from 0x7fff to 0xffff	      
    def position_conversion_to_float(self,position_hex):
      
	position_length = len(position_hex)
	
	float_position=[]                
	
	for i in range(position_length):
	  
	  # Convert position and pid_out to integer values
	    # positive values: from 0 to 32767
	    # negative values: from 32767 to 65535
	    int_position = int(position_hex[i], 0) 
	  
	  # shift negative positions: from -32767 to 0
	    if (int_position >= self.sign_parameter):
	      int_position -= self.twos_complement_shift
	    
	    
	  # get the float value of the position by shifting the point of 8 bits (/2**8)
	    int_position /= 256.0
	  
	  # Add the float position value to the output list
	    float_position = float_position + [int_position]
	return float_position
	
	#print float_position
    
  
  
  
### Conversion of the position from hexadecimal values to decimal values
#  
      # hexadecimal format: 
	# signed integers (twos complement) 16 bits
	# positive values: from 0x0000 to 0x7fff
	# negative values: from 0x7fff to 0xffff  
    def pid_out_conversion_to_float(self,pid_out_hex):
      
	pid_out_length = len(pid_out_hex)
	float_pid_out=[]
	
	for i in range(pid_out_length):
	  
	  # Convert position and pid_out to integer values
	    # positive values: from 0 to 32767
	    # negative values: from 32767 to 65535
	    int_pid_out = int(pid_out_hex[i],0)
	  
	  # shift negative PID out: from -32767 to 0
	    if (int_pid_out >= self.sign_parameter):   
	      int_pid_out -= self.twos_complement_shift
	      
	  # Add the float PID values to output list
	    float_pid_out = float_pid_out + [int_pid_out]
	  
	return float_pid_out
	  
	#print float_pid_out
	
### Sort the data per incresing positions 
#
    def sort_data_per_inc_position(self, position, pid_out):
	position_tmp = position
	pid_out_tmp = pid_out
      #normally pid and position should  have the  same length
	length = min(len(position),len(pid_out))
	position_sorted = []
	pid_out_sorted = []    
	for i in range(length):
	  # look for the smallest position in position_tmp
	    index_min_pos = position_tmp.index(min(position_tmp))
	  # add this smallest position (and the matching pid_out) to sorted position (resp. pid_out) list
	    position_sorted = position_sorted + [position_tmp[index_min_pos]]
	    pid_out_sorted = pid_out_sorted + [pid_out_tmp[index_min_pos]]
	  # delete this smallest position in position_tmp and the matching pid_out in pid_out_tmp
	    position_tmp.pop(index_min_pos)
	    pid_out_tmp.pop(index_min_pos)
	return [position_sorted, pid_out_sorted]
	
	
### Return some joint characteristics
#
      # Inputs: joint name and hand number
		# Caution: the parameter hand_nb must be a string
      # Outputs:
	    # min_angle
	    # max_angle
	    # node_id	
	    # min_osc_angle: position where oscillations appear (with velocity loop)
	    # max_osc_angle: position where oscillations appear (with velocity loop)
      # Those joint characteristics are used to generate the U_map text file 
      
    def joint_characteristics(self, joint_name, hand_nb):
	
      # First finger 
	if (joint_name == "FFJ1"): 
            motor = 'ff0';      min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 90;     node_id = 'node ' + hand_nb + '12' + '0310'
	elif (joint_name == "FFJ2" ):
            motor = 'ff0';      min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 90;     node_id = 'node ' + hand_nb + '12' + '0310'
	elif (joint_name == "FFJ3"):
            motor = 'ff3';      min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 90;     node_id = 'node ' + hand_nb + '13'+'0310'
	elif (joint_name == "FFJ4"):
            motor = 'ff4';      min_osc_angle = -15;     max_osc_angle = 15;
	    min_angle = -25;    max_angle = 25;     node_id = 'node ' + hand_nb + '11' + '0310'
	
      # Medium finger
        
	elif ( joint_name == 'MFJ1'):
            motor = 'mf0';      min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 90;     node_id = 'node ' + hand_nb + '02' + '0310'
	elif ( joint_name == 'MFJ2'):
            motor = 'mf0';      min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 90;     node_id = 'node ' + hand_nb + '02' + '0310'
	elif ( joint_name == 'MFJ3'):
            motor = 'mf3';      min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 90;     node_id = 'node ' + hand_nb + '16' + '0310'
	elif ( joint_name == 'MFJ4'):
            motor = 'mf4';      min_osc_angle = -15;     max_osc_angle = 15;
	    min_angle = -25;    max_angle = 25;     node_id = 'node ' + hand_nb + '01' + '0310'
    
      # Right Finger:
	elif ( joint_name == 'RFJ1'):
            motor = 'rf0';      min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 90;     node_id = 'node ' + hand_nb + '04' + '0310'
	elif ( joint_name == 'RFJ2'):
            motor = 'rf0';      min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 90;     node_id = 'node ' + hand_nb + '04' + '0310'
	elif ( joint_name == 'RFJ3'):
            motor = 'rf3';      min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 90;     node_id = 'node ' + hand_nb + '05' + '0310'
	elif ( joint_name == 'RFJ4'):
            motor = 'rf4';      min_osc_angle = -15;     max_osc_angle = 15;
	    min_angle = -25;    max_angle = 25;     node_id = 'node ' + hand_nb + '03' + '0310'

      # Little finger
	elif ( joint_name == 'LFJ1'):
            motor = 'lf0';      min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 90;     node_id = 'node ' + hand_nb + '09' + '0310'
	elif ( joint_name == 'LFJ2'):
	    motor = 'lf0';      min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 90;     node_id = 'node ' + hand_nb + '09' + '0310'
	elif ( joint_name == 'LFJ3'):
            motor = 'lf3';      min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 90;     node_id = 'node ' + hand_nb + '08' + '0310'
	elif ( joint_name == 'LFJ4'):
            motor = 'lf4';      min_osc_angle = -15;     max_osc_angle = 15;
	    min_angle = -25;    max_angle = 25;     node_id = 'node ' + hand_nb + '10' + '0310'
	elif ( joint_name == 'LFJ5'):
            motor = 'lf5';      min_osc_angle = 10;      max_osc_angle = 35;
	    min_angle = 0;      max_angle = 45;     node_id = 'node ' + hand_nb + '06' + '0310'    

      # Thumb
	elif ( joint_name == 'THJ1'):
            motor = 'th1';      min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 90;     node_id = 'node ' + hand_nb + '14' + '0310'
	elif ( joint_name == 'THJ2'):
            motor = 'th2';      min_osc_angle = -20;     max_osc_angle = 20;
	    min_angle = -30;    max_angle = 30;     node_id = 'node ' + hand_nb + '15' + '0310'
	elif ( joint_name == 'THJ3'):
            motor = 'th3';      min_osc_angle = -10;     max_osc_angle = 10;
	    min_angle = -15;    max_angle = 15;     node_id = 'node ' + hand_nb + '07' + '0310'
	elif ( joint_name == 'THJ4'):
            motor = 'th4';      min_osc_angle = 6;       max_osc_angle = 69;
	    min_angle = 0;      max_angle = 75;     node_id = 'node ' + hand_nb + '19' + '0310'
	elif ( joint_name == 'THJ5'):
            motor = 'th5';      min_osc_angle = -50;     max_osc_angle = 50;
	    min_angle = -60;    max_angle = 60;     node_id = 'node ' + hand_nb + '20' + '0310'

      # Wrist
	elif ( joint_name == 'WRJ1'):
            motor = 'wr1';      min_osc_angle = -40;     max_osc_angle = 30;
	    min_angle = -45;    max_angle = 35;     node_id = 'node ' + hand_nb + '18' + '0310'
	elif ( joint_name == 'WRJ2'):
            motor = 'wr2';      min_osc_angle = -20;     max_osc_angle = 0;
	    min_angle = -30;    max_angle = 10;     node_id = 'node ' + hand_nb + '17' + '0310'
	else:
	  print "The joint is wrong it should be in the following format 'FFJ4'"
	  return 
	  
	return [min_angle, max_angle, node_id, motor, min_osc_angle, max_osc_angle]
	            
### To run a command with a subprocess
#
      # parameters:
	  # command: the command line
    def run_command(self, command):
      # launch the subprocess        
        p = subprocess.Popen(command.split(),stdout=subprocess.PIPE)        
      # allow external program to work
        p.wait()
      # read and return the result to a string
        result_str = p.stdout.read()        
        return result_str

### run a command without waiting for the end of its execution
#
    def run_command_wait_free(self, command):
      # launch the subprocess
        p = subprocess.Popen(command.split(),stdout=subprocess.PIPE)
        return p 
        
### Compute the sign of a list    
#   
      # return:
	  # 1 if the majority of the parameter list elements are stricly positive
	  # 0 if the number of positive and negative values in data_list is the same
	  # 1 if the majority of the parameter list elements are stricly neagtive
    def list_sign(self, data_list):
      # Counter of negative (resp. positive) values
	neg_counter = 0
	pos_counter = 0
      # Computation
	for i in range(len(data_list)):
	    if (data_list[i] > 0.0 ):		
		pos_counter += 1
	    elif (data_list[i] < 0.0):
		neg_counter += 1
      # result
	if (pos_counter > neg_counter):
	    return 1
	elif (pos_counter < neg_counter):
	    return -1
	else:
	    return 0
	    
### Convert the position to the U_map format
#
      # angles: signed float to unsigned integer coded on 16 bits
          # conversion steps: 
                # - *256 gives a signed integer
                # - + 2^15 changes the top bit
                # - conversion to hex considering the values as unsigned integers
    def set_pos_to_u_map_format(self, position):
	u_map_position = []            
        for i in range(len(position)):
	  u_map_position = u_map_position + [hex(2**15 + int(position[i] * 256))]	  
	return u_map_position
	

### Convert the pid output to the U_map format
#
      # PID: signed integer to signed float: 
            # Conversion steps:
                # - /256 (move the point of 8 bits (2**8))
    def set_pid_to_u_map_format(self, pid):
	u_map_pid = []
	for i in range(len(pid)):
	  u_map_pid = u_map_pid + [pid[i]/256.0]	  
      	return u_map_pid
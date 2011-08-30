#!/usr/bin/env python
# -*- coding: utf-8 -*-
import subprocess, string, os
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


### Return some mechanical joint characteristics
#
      # Inputs: joint name and hand number
		# Caution: the parameter hand_nb must be a string
      # Outputs:
	    # min_angle
	    # max_angle
	    # min_osc_angle: position where oscillations appear (with velocity loop)
	    # max_osc_angle: position where oscillations appear (with velocity loop)
    def joint_characteristics(self, joint_name, hand_nb):

      # First finger
	if (joint_name == "FFJ1"):
            min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 85;
	elif (joint_name == "FFJ2" ):
            min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 85;
	elif (joint_name == "FFJ3"):
            min_osc_angle = 5;       max_osc_angle = 79;
	    min_angle = 0;      max_angle = 79;
	elif (joint_name == "FFJ4"):
            min_osc_angle = -15;     max_osc_angle = 15;
	    min_angle = -25;    max_angle = 25;

      # Medium finger

	elif ( joint_name == 'MFJ1'):
            min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 85;
	elif ( joint_name == 'MFJ2'):
            min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 85;
	elif ( joint_name == 'MFJ3'):
            min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 85;
	elif ( joint_name == 'MFJ4'):
            min_osc_angle = -15;     max_osc_angle = 15;
	    min_angle = -25;    max_angle = 25;

      # Right Finger:
	elif ( joint_name == 'RFJ1'):
            min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 85;
	elif ( joint_name == 'RFJ2'):
            min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 85;
	elif ( joint_name == 'RFJ3'):
            min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 85;
	elif ( joint_name == 'RFJ4'):
            min_osc_angle = -15;     max_osc_angle = 15;
	    min_angle = -25;    max_angle = 25;

      # Little finger
	elif ( joint_name == 'LFJ1'):
            min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 85;
	elif ( joint_name == 'LFJ2'):
	    min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 85;
	elif ( joint_name == 'LFJ3'):
            min_osc_angle = 5;       max_osc_angle = 85;
	    min_angle = 0;      max_angle = 85;
	elif ( joint_name == 'LFJ4'):
            min_osc_angle = -15;     max_osc_angle = 15;
	    min_angle = -25;    max_angle = 25;
	elif ( joint_name == 'LFJ5'):
            min_osc_angle = 10;      max_osc_angle = 35;
	    min_angle = 0;      max_angle = 45;

      # Thumb
	elif ( joint_name == 'THJ1'):
            min_osc_angle = 5;       max_osc_angle = 100;
	    min_angle = 0;      max_angle = 110;
	elif ( joint_name == 'THJ2'):
            min_osc_angle = -20;     max_osc_angle = 20;
	    min_angle = -30;    max_angle = 30;
	elif ( joint_name == 'THJ3'):
            min_osc_angle = -10;     max_osc_angle = 10;
	    min_angle = -15;    max_angle = 15;
	elif ( joint_name == 'THJ4'):
            min_osc_angle = 6;       max_osc_angle = 69;
	    min_angle = 0;      max_angle = 75;
	elif ( joint_name == 'THJ5'):
            min_osc_angle = -50;     max_osc_angle = 50;
	    min_angle = -60;    max_angle = 60;

      # Wrist
	elif ( joint_name == 'WRJ1'):
            min_osc_angle = -40;     max_osc_angle = 30;
	    min_angle = -45;    max_angle = 35;
	elif ( joint_name == 'WRJ2'):
            min_osc_angle = -20;     max_osc_angle = 0;
	    min_angle = -30;    max_angle = 10;
	else:
	  print "The joint is wrong it should be in the following format 'FFJ4'"
	  return

	return [min_angle, max_angle, min_osc_angle, max_osc_angle]

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


### Write the U_map values in a text file
#
      # needs the direction to set the right table name
    def write_u_map_in_text_file(self, u_map_position, u_map_pid_out, output_file, node_id, direction):
      # Text file name
          print "writing to : ", output_file
          directory = "/".join(output_file.split("/")[:-1])
          print "   -> ",directory
          self.ensure_dir(directory)

          fout = open(output_file, 'w')


      # Node id
          fout.write(node_id)
          fout.write('\n')
          fout.write('\n')

          if (direction == "forward"):
            # POSITIVE DIRECTION TABLE
                fout.write('#FRICTION_MAP_POSITIVE_DIRECTION')
                fout.write('\n')
                fout.write('tbl 5')
                fout.write('\n')
          else:
            # NEGATIVE DIRECTION TABLE
                fout.write('#FRICTION_MAP_NEGATIVE_DIRECTION')
                fout.write('\n')
                fout.write('tbl 6')
                fout.write('\n')

          for j in range(len(u_map_position)):
              fout.write(str(u_map_position[j]))
              fout.write(' @ ')
              fout.write(str(u_map_pid_out[j]))
              fout.write('\n')
          fout.write('\n')






### Write forward AND backward U_map values in ONE text file
#
      # Normally this function is unused but can be useful for debuging
    def write_forward_and_backward_u_map_in_one_text_file(self, u_map_position_forward, u_map_pid_out_forward, u_map_position_backward, u_map_pid_out_backward, output_file, node_id):

      # Text file name
          fout = open(output_file, 'w')

      # Node id
          fout.write(node_id)
          fout.write('\n')
          fout.write('\n')

      # POSITIVE DIRECTION TABLE
          fout.write('#FRICTION_MAP_POSITIVE_DIRECTION')
          fout.write('\n')
          fout.write('tbl 5')
          fout.write('\n')

          for j in range(len(u_map_position_forward)):
              fout.write(str(u_map_position_forward[j]))
              fout.write(' @ ')
              fout.write(str(u_map_pid_out_forward[j]))
              fout.write('\n')
          fout.write('\n')


      # NEGATIVE DIRECTION TABLE
          fout.write('#FRICTION_MAP_NEGATIVE_DIRECTION')
          fout.write('\n')
          fout.write('tbl 6')
          fout.write('\n')

          for j in range(len(u_map_position_backward)):
              fout.write(str(u_map_position_backward[j]))
              fout.write(' @ ')
              fout.write(str(u_map_pid_out_backward[j]))
              fout.write('\n')

    def ensure_dir(self, f):
        try:
            os.makedirs(f)
        except:
            pass

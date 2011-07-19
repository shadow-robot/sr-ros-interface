#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################################################
# name: U_Map_Data_Acquisition
# purpose: Collect data from the hand for U_map computation
####################################################################################################
import roslib; roslib.load_manifest('sr_friction_compensation')
import rospy

import string

from sr_friction_compensation.utils.utilitarian import Utilitarian
from sr_friction_compensation.filters.u_map_filter_1o import U_Map_Filter_1o
from sr_friction_compensation.filters.u_map_filter_single_pos import U_Map_Filter_Single_Pos


class U_Map_Data_Acquisition(object):

### Constructor:
#

  def __init__(self, joint_name, hand_nb, direction):
    self.stopped = True
    self.hand_nb = hand_nb
    self.joint_name = joint_name
    self.direction = direction

    # class instances declaration
    self.utilitarian = Utilitarian()
    self.filter_1o = U_Map_Filter_1o()
    self.filter_single_pos = U_Map_Filter_Single_Pos()

    # Final values for U_map computation
    self.position = []
    self.pid_out = []
    
    # Float values
    self.position_uniq = []
    self.pid_out_uniq = []

    # data filtered with a first order filtered
    self.position_filtered_1o = []
    self.pid_out_filtered_1o = []

    # Sorted data
    self.sorted_position = []
    self.sorted_pid_out = []

    # data with single positions
    self.position_filtered_single_pos = []
    self.pid_out_filtered_single_pos = []

### Run measurements
#
  def measurement_process(self, P, I, D, shift):
    pass

### Prepare and run data treatment
#
  def run_data_acquisition(self, P, I, D, shift):
    pass


### Prepare data for U_map computation
#
      # Outputs:
          # position
          # pid_out
      # Actions:
          # Delete repeated tuples (position,pid_out)
          # Convert from hexadecimal to float for position and integers for pid output
          # Filter the data with a first order filter
          # Sort data per increasing position
          # Delete repeated positions so that they appear only once with the average PID value  
  def data_treatment(self, position_float, pid_out_float):
      # Delete tuples (positions, pid_out) recorded several times
    [self.position_uniq, self.pid_out_uniq] = self.utilitarian.delete_repeated_tuples(position_float, pid_out_float)

     
      # Filter the data with a first order filter
    self.position_filtered_1o = self.filter_1o.filter_fun(self.position_uniq, 0.05)
    self.pid_out_filtered_1o = self.filter_1o.filter_fun(self.pid_out_uniq, 0.5)

    # Display the filtered data in columns
          #print "\n", "Filtered data:"
          #for i in range(len(self.position_filtered_1o)):
            #print i, "\t",self.position_filtered_1o[i], "\t", self.pid_out_filtered_1o[i]

    # Sorting the data per increasing position
    [self.sorted_position, self.sorted_pid_out] = self.utilitarian.sort_data_per_inc_position(self.position_filtered_1o, self.pid_out_filtered_1o)


    # Display the sorted data in columns
          #print "\n","Sorted data:"
          #for i in range(len(self.sorted_position_forward)):
            #print i, "\t",self.sorted_position[i], "\t", self.sorted_pid_out[i]

    # Delete the repeated position so that they appear only once with the average pid value
    [self.position_filtered_single_pos, self.pid_out_filtered_single_pos] = self.filter_single_pos.filter_fun(self.sorted_position, self.sorted_pid_out)


    # Display the single positions data in columns
    print "\n","Single positions data:"
    for i in range(len(self.position_filtered_single_pos)):
      print i, "\t",self.position_filtered_single_pos[i], "\t", self.pid_out_filtered_single_pos[i]

    # Return data ready for U_map computation
    return [self.position_filtered_single_pos, self.pid_out_filtered_single_pos]

  def stop(self):
    self.stopped = True

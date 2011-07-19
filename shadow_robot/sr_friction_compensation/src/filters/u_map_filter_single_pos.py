#!/usr/bin/env python
# -*- coding: utf-8 -*-

from u_map_filter import U_Map_Filter 

class U_Map_Filter_Single_Pos(U_Map_Filter):
  
### Constructor
#
  def __init__(self):
      U_Map_Filter.__init__(self)
      self.position_filtered = []
      self.pid_out_filtered = []


### filter function
#
  # Action: 
      # Delete the repeated position so that they appear only once with the average pid value 
  # CAUTION: input data must be sorted per increasing position 
  # note:
      # I changed the name of the first parameter (it was data in the super class)
            #def filter_fun(self, data, data_2):
    
  def filter_fun(self, position, pid_out):
    # declaration of the outputs
      self.position_filtered = []
      self.pid_out_filtered = []
    # variables for PID average computation
      tmp_sum = 0.0
      N = 0.0
      data_length = min(len(position), len(pid_out)) # Normally position and pid_out have the same length
      for i in range(data_length - 1):
	# if 2 consecutive positions are equals it computes the average variables
	  if (position[i] == position[i+1]):
	      tmp_sum += pid_out[i]
	      N = N+1.0;
	# when the next position becomes different it computes the actual average PID value 
	  elif ( N!= 0.0):
	      tmp_sum += pid_out[i]
	      N = N+1.0;
	      self.pid_out_filtered = self.pid_out_filtered + [tmp_sum/N]
	      self.position_filtered = self.position_filtered + [position[i]]
	      tmp_sum = 0.0
	      N = 0.0
	# if consecutive positions are different it copies position and pid_out in output lists
	  else:
	      self.pid_out_filtered = self.pid_out_filtered + [pid_out[i]]
	      self.position_filtered = self.position_filtered + [position[i]]	    
    # return the filtered data
      return [self.position_filtered, self.pid_out_filtered]
    
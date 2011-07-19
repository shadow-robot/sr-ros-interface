#!/usr/bin/env python
# -*- coding: utf-8 -*-

from u_map_filter import U_Map_Filter 

class U_Map_Filter_1o(U_Map_Filter):
  
### Constructor
#
    # a is the filter coefficient which equation is: 
	# Yn = a*Yn + (1-a)*Yn-1    [1]
  def __init__(self): 
      U_Map_Filter.__init__(self)
      self.a = []
      
      
### filter function
#
  def filter_fun(self, data, a):
      self.data = data
      self.a = a
      self.data_length = len(self.data)      
      self.data_filtered = [self.data[0]]
      for i in range(self.data_length-1): 
	# Compute the filtered data using equation [1]
	  filtered_value = self.data[i+1]*self.a + (1-self.a)*self.data[i]
	# Add it to the output list
	  self.data_filtered = self.data_filtered + [filtered_value]
    # Return the filtered data
      return self.data_filtered
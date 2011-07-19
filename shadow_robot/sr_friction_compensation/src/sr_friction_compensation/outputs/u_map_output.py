#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Imports list
from __future__ import division

import roslib; roslib.load_manifest('sr_friction_compensation')
import rospy

from sr_friction_compensation.utils.utilitarian import Utilitarian

class U_Map_Output(object):
  
### Constructor 
#    
    def __init__(self, joint_name, hand_number, lib):
        self.joint_name = joint_name
	self.utilitarian = Utilitarian()
	self.lib = lib
	self.hand_number = hand_number
	
        [self.min_joint_angle, self.max_joint_angle,min_osc_angle, max_osc_angle] = self.utilitarian.joint_characteristics(self.joint_name, self.hand_number)
      
### U_map computation
#
	# Choose n equidistant points among the hand data
	# Inputs:
	    # position: position list
	    # pid: pid_out list
	    # n: number of U_map tables points	    
	# Outputs:
	    # u_map_position: u_map position values
	    # u_map_pid: u_map pid out values
    def u_map_computation(self, position, pid, n):
	  # Number of points
	      point_nb = n;
			  
	  # Computation of the PID sign
	      pid_sign = self.utilitarian.list_sign(pid)

	  # minimum and maximum measured values
	      min_position = min(position)
	      max_position = max(position)
	      min_pid = min(pid)
	      max_pid = max(pid)

	  # U_map points computation (choose point_nb equidistant points among measured data)     
	      u_map_position = []
	      u_map_pid = []
	      reference_angles = []
	      step = abs(min_position - max_position)/(point_nb-1)
	    # reference_angles contains the theoretical equidistant points
	      reference_angles = reference_angles + [position[0]]
	      for i in range(point_nb-1): # -1 because the first value is already set
		  reference_angles = reference_angles + [reference_angles[i] + step]
	    # then this loop computes the measured points closest to theoretical equidistant points   
	      for i in range(point_nb):	      
		  position_diff = []
		  for j in range(len(position)):
		      position_diff = position_diff + [abs(position[j] - reference_angles[i])]
		  
		  index = position_diff.index(min(position_diff))	      		  
		  u_map_position = u_map_position + [position[index]]
		# it makes sure that the pid values have the same sign
		  if (pid_sign >= 0):
		      u_map_pid = u_map_pid + [max(0,pid[index])]
		  else:
		      u_map_pid = u_map_pid + [min(0,pid[index])]	 	  
	      
	      
	  # Display the u_map values in columns
	      print "\n","temporary U_map:"	
	      for i in range(point_nb):	    
		print i, "\t", u_map_position[i], "\t", u_map_pid[i]
	      
	      
	  # Set limit u_map positions to min and max position (using linear interpolation)
	    # Firmware limit values for PID
	      Firmware_min_PID = -1023;
	      Firmware_max_PID = 1023;    
            
              if (u_map_position[point_nb -1] < self.max_joint_angle):
                  m = (u_map_pid[point_nb-1]-u_map_pid[point_nb-2])/(u_map_position[point_nb-1]-u_map_position[point_nb-2])
                  if (pid_sign >= 0):                   
                      u_map_pid[point_nb-1]= max(0,min(Firmware_max_PID,m*(self.max_joint_angle-u_map_position[point_nb-1])+u_map_pid[point_nb-1]))
                  else:                   
                      u_map_pid[point_nb-1]= min(0,max(Firmware_min_PID,m*(self.max_joint_angle-u_map_position[point_nb-1]) + u_map_pid[point_nb-1]))
                  
                  u_map_position[point_nb -1] = self.max_joint_angle                  
              
              if (u_map_position[1]>self.min_joint_angle):
                  m = (u_map_pid[1]-u_map_pid[0])/(u_map_position[1]-u_map_position[0])
                  if (pid_sign >= 0):
                      u_map_pid[0]= max(0, min(Firmware_max_PID, m * (self.min_joint_angle - u_map_position[0]) + u_map_pid[0]))
                  else:                    
                      u_map_pid[0]= min(0, max(Firmware_min_PID, m * (self.min_joint_angle - u_map_position[0]) + u_map_pid[0]))         
                  u_map_position[0]=self.min_joint_angle
	    
	    # Display the u_map values in columns
	      print "\n","U_map:"	
	      for i in range(point_nb):	    
		print i, "\t", u_map_position[i], "\t", u_map_pid[i]
	
           # Conversion to the U_map format
              final_u_map_position = self.utilitarian.set_pos_to_u_map_format(u_map_position)
              final_u_map_pid = self.utilitarian.set_pid_to_u_map_format(u_map_pid)      

	  # return the final values
	      return [u_map_position, u_map_pid ,final_u_map_position, final_u_map_pid]
	      
### Write to file, then sendcal
#
    def send_umap(self,u_map_position, u_map_pid_out):
        pass
      
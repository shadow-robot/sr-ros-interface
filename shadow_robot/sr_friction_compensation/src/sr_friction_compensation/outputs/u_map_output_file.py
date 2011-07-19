#!/usr/bin/env python
# -*- coding: utf-8 -*-

from u_map_output import U_Map_Output

class U_Map_Output_File(U_Map_Output):
  
### Constructor
#        
    def __init__(self, joint_name, hand_number, direction):
	U_Map_Output.__init__(self, joint_name, hand_number)
        
        self.direction = direction
        self.output_file = self.joint_name + "_" + self.direction +"_friction_compensation.txt"
        
        
        
### Write the U_map values in a text file
#
      # needs the direction to set the right table name
    def write_u_map_in_text_file(self, u_map_position, u_map_pid_out ):
      # Text file name	  
	  fout = open(self.output_file, 'w')
	  
	  
      # Node id      
	  fout.write(self.node_id)
	  fout.write('\n')
	  fout.write('\n')
	  
	  if (self.direction == "forward"):
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
    def write_forward_and_backward_u_map_in_one_text_file(self, u_map_position_forward, u_map_pid_out_forward, u_map_position_backward, u_map_pid_out_backward ):

      # Text file name
	  fout = open(self.output_file, 'w')
	  
      # Node id      
	  fout.write(self.node_id)
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
	  
### Send the U_map table to the firmware
#
      # Parameter:
	  # u_map_file_name: file containing the u_map table
    def send_u_map_to_firmware(self):	  
	  command = "sendcal "+ self.output_file   
	  answer = self.utilitarian.run_command(command)
  
### Write to file, then sendcal            
#    
    def send_umap(self,u_map_position, u_map_pid_out):        
      # Generate u_map file
        self.write_u_map_in_text_file(u_map_position, u_map_pid_out)
      # Send u_map file
        #self.send_u_map_to_firmware()
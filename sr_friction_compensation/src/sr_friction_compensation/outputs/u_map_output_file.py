#!/usr/bin/env python
# -*- coding: utf-8 -*-

from u_map_output import U_Map_Output

class U_Map_Output_File(U_Map_Output):

### Constructor
#
    def __init__(self, joint_name, hand_number, lib, direction):
	U_Map_Output.__init__(self, joint_name, hand_number, lib)
        self.direction = direction


### Write to file, then sendcal
#
    def send_umap(self,u_map_position, u_map_pid_out):
      # Send u_map file
        self.lib.send_u_map_to_firmware(u_map_position, u_map_pid_out, self.direction, self.joint_name, self.hand_number)

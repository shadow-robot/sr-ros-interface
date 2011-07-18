#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2011 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#By: Emilie JEAN-BAPTISTE
##Date:12 Juillet 2011

import roslib; roslib.load_manifest('sr_hand')
import rospy
from std_msgs.msg import Float32MultiArray, Float32

class Display_RXPLOT_Service(object):
      def __init__(self,joint_name):
	  self.publisher_for_rxplot=rospy.Publisher("srh/rxplot_fitness_evolution_online_"+joint_name,Float32)
	  return
	  
      def send_data_on_rxplot(self,fitness_vector_mean):
	  """
	  Using data from fitness vector mean and send it to RXPLOT
	  @return: nothing
	  """
	  pub=self.publisher_for_rxplot
	  msg=Float32()
	  for mean_val in fitness_vector_mean:
	      msg.data=mean_val
	      pub.publish(msg)
	      rospy.sleep(1)
	      
	  return
	  


	      
	  
	  
#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This is an example showing how to publish command messages to the hand.
"""


import roslib; roslib.load_manifest('sr_hand')
import rospy

from sr_hand.msg import joint, sendupdate, contrlr

def talker():
    """
    The Publisher publishes to the different topics on which the sr_subscriber subscribes. It sends commands to 
    the hand.
    
    Please set the message you want to test.
    """
    test_what = "sendupdate" # choose sendupdate or contrlr
    
    """
    Test the sendupdate command
    """
    if test_what == "sendupdate":
        pub1 = rospy.Publisher('sr_arm/sendupdate', sendupdate)
        rospy.init_node('shadowhand_command_publisher_python')

        new_target = 30
    
        data_to_send = [ joint(joint_name="trunk_rotation", joint_target=new_target),
                         joint(joint_name="shoulder_rotation", joint_target=new_target), 
                         joint(joint_name="elbow_abduction", joint_target=new_target) ]
    
        pub1.publish(sendupdate(len(data_to_send), data_to_send))
        
    """
    Tests the contrlr command
    """
    if test_what == "contrlr":
        pub2 = rospy.Publisher('contrlr', contrlr)
    
        data_to_send = ["p:0","i:0"]
        
        pub2.publish( contrlr( contrlr_name="smart_motor_ff2" , 
                               list_of_parameters = data_to_send, 
                               length_of_list = len(data_to_send) ) )
    


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass

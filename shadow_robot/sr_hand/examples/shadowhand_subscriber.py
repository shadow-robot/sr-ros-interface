#!/usr/bin/env python

"""
This is a simple subscriber example, subscribing to the srh/shadowhand_data topic
"""

import roslib; roslib.load_manifest('sr_hand')
import rospy
from sr_hand.msg import joints_data, joint

def callback(joint_data):
    """
    The callback function for the topic srh/shadowhand_Data. It just displays the received information on the console.
    
    @param joint_data: the message containing the joint data.
    """
    for joint in joint_data.joints_list:
        rospy.loginfo(rospy.get_name()+"[%s] : Pos = %f | Target = %f | Temp = %f | Current = %f",
                      joint.joint_name, joint.joint_position, joint.joint_target, joint.joint_temperature, 
                      joint.joint_current)

def listener():
    """
    Initialize the ROS node and the topic to which it subscribes.
    """
    rospy.init_node('shadowhand_subscriber_python', anonymous=True)
    rospy.Subscriber("srh/shadowhand_data", joints_data, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

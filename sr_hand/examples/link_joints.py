#!/usr/bin/env python
import roslib; roslib.load_manifest('sr_hand')
import rospy
from sr_hand.msg import sendupdate, joint, joints_data
from sensor_msgs.msg import *

parent_name = "FFJ3"
child_name = "MFJ3"


def callback(data):
    """
    The callback function: called each time a message is received on the 
    topic /srh/shadowhand_data

    @param data: the message
    """
    message=[]
    if data.joints_list_length == 0:
        return
    # loop on the joints in the message
    for joint_data in data.joints_list:
        # if its the parent joint, read the target and send it to the child
        if joint_data.joint_name == parent_name:
            message.append(joint(joint_name=child_name, joint_target=joint_data.joint_target))
    
    # publish the message to the /srh/sendupdate topic.
    pub = rospy.Publisher('srh/sendupdate', sendupdate) 
    pub.publish(sendupdate(len(message), message))

def listener():
    """
    The main function
    """
    # init the ros node
    rospy.init_node('joints_link_test', anonymous=True)

    # init the subscriber: subscribe to the topic 
    # /srh/shadowhand_data, using the callback function
    # callback()
    rospy.Subscriber("srh/shadowhand_data", joints_data, callback)

    # subscribe until interrupted
    rospy.spin()


if __name__ == '__main__':
    listener()    


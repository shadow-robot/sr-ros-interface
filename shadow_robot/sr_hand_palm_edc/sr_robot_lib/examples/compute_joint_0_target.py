#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_robot_lib')
import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class Merger(object):
    def __init__(self):
        self.subscriber = rospy.Subscriber("/joint_states", JointState, self.callback)
        self.publisher = rospy.Publisher("/lfj0_target", Float64)

    def callback(self,data):
        msg_to_send = Float64()

        msg_to_send.data = data.position[17] + data.position[18]

        self.publisher.publish(msg_to_send)

if __name__ == '__main__':
    rospy.init_node("merger", anonymous=True)
    merger = Merger()
    rospy.spin()





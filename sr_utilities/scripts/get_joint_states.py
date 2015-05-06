#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from sr_utilities.srv import getJointState
import thread

RATE = 100

class GetJointState(object):
    def __init__(self):
        rospy.init_node('get_joint_state_service', anonymous=True)

        self.subs_1 = rospy.Subscriber("/joint_states", JointState, self.callback1)
        self.serv = rospy.Service('/getJointState', getJointState, self.getJointStateCB)

        self.joint_state_msg = JointState()

        self.mutex = thread.allocate_lock()

        r = rospy.Rate(RATE)
        while not rospy.is_shutdown():
            r.sleep()

    def callback1(self, data):
        self.joint_state_msg = data

    def getJointStateCB(self,req):
        res = self.joint_state_msg
        return res

if __name__ == '__main__':
    service = GetJointState()

#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
import thread


class MergeMessages:
    def __init__(self):
        rospy.init_node('target_joint_state_merger', anonymous=True)
        self.subs_1 = rospy.Subscriber("/srh/target/joint_states", JointState, self.callback1)
        self.subs_2 = rospy.Subscriber("/sr_arm/target/joint_states", JointState, self.callback2)

        self.pub = rospy.Publisher("/targets/joint_states", JointState)

        self.msg_1_received = False
        self.msg_2_received = False

        self.joint_state_msg = JointState()

        self.mutex = thread.allocate_lock()

        rospy.spin()

    def callback1(self, data):
        self.mutex.acquire()
        if self.msg_1_received == True:
            self.mutex.release()
            return

        self.msg_1_received = True

	self.joint_state_msg.header.stamp = rospy.Time.now()

        tmp = data.name
        self.joint_state_msg.name = tmp

        tmp = data.position
        self.joint_state_msg.position = tmp

        tmp = data.velocity
        self.joint_state_msg.velocity = tmp

        tmp = data.effort
        self.joint_state_msg.effort = tmp

	self.mutex.release()

    def callback2(self, data):
        self.mutex.acquire()

        self.msg_2_received = True

	self.joint_state_msg.header.stamp = rospy.Time.now()

        tmp = self.joint_state_msg.name
        tmp += data.name
        self.joint_state_msg.name = tmp

        tmp = self.joint_state_msg.position
        tmp += data.position
        self.joint_state_msg.position = tmp

        tmp = self.joint_state_msg.velocity
        tmp += data.velocity
        self.joint_state_msg.velocity = tmp

        tmp =  self.joint_state_msg.effort
        tmp += data.effort
        self.joint_state_msg.effort = tmp

        if self.msg_1_received == True:
            self.pub.publish(self.joint_state_msg)
            self.msg_1_received = False
            self.msg_2_received = False
            self.joint_state_msg = JointState()

        self.mutex.release()


if __name__ == '__main__':
    merger = MergeMessages()

#!/usr/bin/env python
# This node combines 5 virtual touch sensors into a ShadowPST message compatible with etherCAT hand

import rospy
from std_msgs.msg import Float64
from sr_robot_msgs.msg import ShadowPST
import thread


class MergeMessages(object):
    def __init__(self):
        rospy.init_node('ShadowPST_publisher', anonymous=True)
        self.ff_sub=rospy.Subscriber('/sr_tactile/touch/ff',Float64,self.ff_cb)
        self.mf_sub=rospy.Subscriber('/sr_tactile/touch/mf',Float64,self.mf_cb)
        self.rf_sub=rospy.Subscriber('/sr_tactile/touch/rf',Float64,self.rf_cb)
        self.lf_sub=rospy.Subscriber('/sr_tactile/touch/lf',Float64,self.lf_cb)
        self.th_sub=rospy.Subscriber('/sr_tactile/touch/th',Float64,self.th_cb)
        self.rate = rospy.Rate(25.0)
        self.pub = rospy.Publisher("/tactile", ShadowPST)
        self.pst=[0.0,0.0,0.0,0.0,0.0]
        self.mutex = thread.allocate_lock()

    def ff_cb(self, msg):
        self.mutex.acquire()
        self.pst[0]=msg.data
        self.mutex.release()

    def mf_cb(self, msg):
        self.mutex.acquire()
        self.pst[1]=msg.data
        self.mutex.release()

    def rf_cb(self, msg):
        self.mutex.acquire()
        self.pst[2]=msg.data
        self.mutex.release()

    def lf_cb(self, msg):
        self.mutex.acquire()
        self.pst[3]=msg.data
        self.mutex.release()

    def th_cb(self, msg):
        self.mutex.acquire()
        self.pst[4]=msg.data
        self.mutex.release()

    def shadowpst_publisher(self):
        pst_state_msg=ShadowPST()
        pst_state_msg.temperature=[0,0,0,0,0,0]
        pressure=[]
        self.mutex.acquire()
        for i in range(0,5):
            pressure.append(self.pst[i]*100)
        pst_state_msg.pressure=pressure
        #print pst_state_msg.pressure
        self.mutex.release()

        pst_state_msg.header.stamp = rospy.Time.now()
        self.pub.publish(pst_state_msg)

if __name__ == '__main__':
    merger = MergeMessages()
    while not rospy.is_shutdown():
        merger.shadowpst_publisher()
        merger.rate.sleep()


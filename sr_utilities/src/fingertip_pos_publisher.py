#!/usr/bin/env python
import roslib;roslib.load_manifest('sr_utilities')
import rospy
import math
import tf
from geometry_msgs.msg import Vector3

if __name__ == '__main__':
	rospy.init_node('tf_tippos_publisher')
	listener = tf.TransformListener()
	
	tip_pos_pub= rospy.Publisher('fftip/position/', Vector3)
	rate = rospy.Rate(10.0)
	while not rospy.is_shutdown():
		try:
			(trans,rot) = listener.lookupTransform('/palm', '/fftip', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException):
			continue
		tip_pos_pub.publish(Vector3(trans[0],trans[1],trans[2]))
		rate.sleep()















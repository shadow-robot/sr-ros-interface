#!/usr/bin/env python
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
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
#

## Test the publish subscribe frequency of the hand and arm
## while publishing targets at a given frequency

PKG  = 'sr_tests'
NAME = 'hztest'

import roslib
roslib.load_manifest(PKG)
roslib.load_manifest('rostest')

from sr_robot_msgs.msg import sendupdate, joint
import rospy, rostest
import sys, unittest
import time

MIN_MSGS        = 1
TEST_TIMEOUT    = 360.0

class SRHzTest(unittest.TestCase):
    """
    """
    hz_target = None
    hz_tol    = None
    msg_count = 0

    started    = False
    start_time = 0
    end_time   = 0
    finished   = False

    target_percentage = 0
    target_increment  = 0.05

    def __init__(self, *args):
        """
        """
        rospy.init_node(NAME, anonymous=True)

        self.joints_dict = {"FFJ0": {"min": 0, "max": 90, "current":0},
                            "FFJ3": {"min": 0, "max": 90, "current":0},
                            "MFJ0": {"min": 0, "max": 90, "current":0},
                            "MFJ3": {"min": 0, "max": 90, "current":0},
                            "RFJ0": {"min": 0, "max": 90, "current":0},
                            "RFJ3": {"min": 0, "max": 90, "current":0},
                            "LFJ0": {"min": 0, "max": 90, "current":0},
                            "LFJ3": {"min": 0, "max": 90, "current":0}
                            }

        super(SRHzTest, self).__init__(*args)
        self.success = False

    def test_hz(self):
        self.topic_name = rospy.get_param('~topic', '/srh/shadowhand_data')
        self.hz_target = rospy.get_param('~hz',20);
        self.hz_tol = rospy.get_param('~hzerror',1);
        self.test_duration = rospy.get_param('~test_duration',5);
        rospy.Subscriber(self.topic_name, rospy.AnyMsg, self.callback)

        freq_tmp = rospy.get_param('~publish_frequency',1000);
        self.publish_freq = rospy.Rate(freq_tmp)
        self.topic_pub    = rospy.get_param('~publish_topic', '/srh/sendupdate')
        self.pub = rospy.Publisher(self.topic_pub, sendupdate)

        self.wait_for_topic = rospy.get_param('~wait_for_topic', '')
        if self.wait_for_topic != '':
            rospy.wait_for_message(self.wait_for_topic, rospy.AnyMsg)
            time.sleep(1.0)
            print "[TEST] starting the test"

        timeout_t = time.time() + self.test_duration + TEST_TIMEOUT
        print "Timeout: ", timeout_t
        while not rospy.is_shutdown() and not self.success and not self.finished and time.time() < timeout_t:
            self.publish_sendupdate()
            self.publish_freq.sleep()

        print " realtime:",time.time()," simtime:",rospy.get_rostime().to_sec()," time elapsed:",(self.end_time - self.start_time)," msg count:",self.msg_count

        self.assert_(self.success)

    def publish_sendupdate(self):
        """

        Arguments:
        - `self`:
        """
        data_to_send = []

        for joint_name in self.joints_dict.keys():
            tmp_target = self.joints_dict[joint_name]["min"] + \
                (self.joints_dict[joint_name]["max"] - self.joints_dict[joint_name]["min"]) \
                * self.target_percentage
            tmp_target = 0
            data_to_send.append(joint(joint_name=joint_name, joint_target=tmp_target))

        self.target_percentage += self.target_increment
        if self.target_increment > 0 and self.target_percentage >= 1.0:
            self.target_increment = - self.target_increment
        else:
            if self.target_increment < 0 and self.target_percentage <= 0.0:
                self.target_increment = - self.target_increment

        self.pub.publish(sendupdate( len(data_to_send), data_to_send ) )

    def callback(self, msg):
        self.msg_count += 1

        if not self.started:
          self.start_time = rospy.get_rostime().to_sec()
          print " got first message at: ",self.start_time, " sec"
          self.started = True

        self.end_time = rospy.get_rostime().to_sec()

        if (self.end_time - self.start_time) > self.test_duration:
          self.finished = True

        if (self.end_time - self.start_time) > 0.0:
          cur_hz = self.msg_count / (self.end_time - self.start_time)
        else:
          cur_hz = 0

        if abs(cur_hz - self.hz_target) < self.hz_tol:
          self.success = True
        else:
          self.success = False

if __name__ == '__main__':
    print sys.argv[0]
    print sys.argv
    rostest.run(PKG, sys.argv[0], SRHzTest, sys.argv)



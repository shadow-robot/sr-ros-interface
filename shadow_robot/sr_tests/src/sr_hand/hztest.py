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

## Test the publish and subscribe frequency of the hand and arm:
##

PKG  = 'sr_tests'
NAME = 'hztest'

import roslib
roslib.load_manifest(PKG)
roslib.load_manifest('rostest')

import sys, unittest

from sr_robot_msgs.msg import sendupdate, joints_data

class MyHzTest(unittest.TestCase):
    """
    """
    hz_target = None
    hz_tol    = None

    def __init__(self, *args):
        """
        """
        super(MyHzTest, self).__init__(*args)
        self.success = False

        #init node and params
        rospy.init_node(NAME, anonymous=True)
        self.topic_name = rospy.get_param('~topic', '/srh/shadowhand_data')

        self.assert_(True)

if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], MyHzTest, sys.argv)



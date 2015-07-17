#!/usr/bin/env python
#
# Copyright 2014 Shadow Robot Company Ltd.
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

import rospy

class HandConfig(object):
    def __init__(self, mapping, joint_prefix):
        """

        """
        self.mapping = mapping
        self.joint_prefix = joint_prefix

class HandFinder(object):
    """
    The HandFinder is a utility library for detecting Shadow Hands running on the system.
    The idea is to make it easier to write generic code, using this library to handle prefixes, joint prefixes etc...
    """

    def __init__(self):
        """
        Parses the parameter server to extract the necessary information.
        """
        hand_parameters = rospy.get_param("hand")
        self.hand_config = HandConfig(hand_parameters["mapping"], hand_parameters["joint_prefix"])

    def get_hand_parameters(self):
        return self.hand_config
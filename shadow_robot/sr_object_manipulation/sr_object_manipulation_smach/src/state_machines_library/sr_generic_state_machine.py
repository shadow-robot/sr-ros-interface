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

import roslib; roslib.load_manifest('sr_object_manipulation_smach')
import rospy

import smach
import smach_ros

class SrGenericStateMachine(object):
    """
    This is a generic parent from which every state machine used for interacting
    with the Shadow Robot's stack inherits.
    """

    state_machine = None

    def __init__(self, sm_name, sm_outcomes = ['success', 'failed'], sm_output_keys = [], sm_input_keys = []):
        """
        Initializes the state machine with the given parameters.

        @sm_outcomes: the state machine outcome, default is  ['success', 'failed']
        """
        self.state_machine = smach.StateMachine(outcomes = sm_outcomes,
                                                output_keys = sm_output_keys,
                                                input_keys = sm_input_keys)


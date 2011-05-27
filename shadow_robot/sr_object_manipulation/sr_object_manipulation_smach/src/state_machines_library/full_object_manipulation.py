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

from sr_generic_state_machine import SrGenericStateMachine
from vision.sm_object_detection_and_recognition import SrObjectDetectionAndRecognitionStateMachine


class Starting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failed'])

    def execute(self, userdata):
        return 'success'

class Finishing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failed'],
                             input_keys=['objects_data_in'])

    def execute(self, userdata):
        return 'success'

class SrFullObjectManipulationStateMachine(SrGenericStateMachine):
    """
    Main class containing the complete state machine for the
    object manipulation stack using Shadow Robot's hardware.
    """
    introspection_server = None

    def __init__(self):
        """
        Initializes the object manipulation state machine using the state machines
        defined in the state_machines_library folder.
        """
        SrGenericStateMachine.__init__(self, sm_name="full_object_manipulation")

        self.object_detection = SrObjectDetectionAndRecognitionStateMachine()
        self.introspection_server = smach_ros.IntrospectionServer("FullObjectManipulation", self.state_machine, "/FullObjectManipulation")
        with self.state_machine:
            self.state_machine.add('Starting', Starting(),
                                   transitions={'success':'DetectingAndRecognizingObjects'})
            self.state_machine.add('DetectingAndRecognizingObjects', self.object_detection.state_machine,
                                   transitions={'success':'Finishing'} )
            self.state_machine.add('Finishing', Finishing(),
                                   remapping={'objects_data_in':'objects_data_out'})

        self.introspection_server.start()

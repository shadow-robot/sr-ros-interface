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
from pickup.sm_pickup_object import SrPickupObjectStateMachine
from place.sm_place_object import SrPlaceObjectStateMachine

class Starting(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failed'])

    def execute(self, userdata):
        return 'success'

class Finishing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['restart','success'],
                             input_keys=['objects_data_in'])

    def execute(self, userdata):
        print ""
        print ""
        print "----------------------------------"

        try:
            result = raw_input("Do you want to restart? [y/N]")
            if result == "y" or result == "Y":
                print " -> RESTARTING..."
                return 'restart'
            else:
                print " -> DONE"
                return 'success'
        except:
            return 'success'

        return 'restart'

class UserChooseObject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failed'],
                             input_keys=['objects_data_in',
                                         'graspable_objects_in',
                                         'graspable_objects_names_in',
                                         'collision_support_surface_name_in'],
                             output_keys=['graspable_object_out',
                                          'graspable_object_name_out',
                                          'collision_support_surface_name_out'])
    def execute(self, userdata):
        print ""
        print ""
        print "----------------------------------"

        for index,obj in enumerate(userdata.objects_data_in):
            print "[", index,"]", obj.model_description

        try:
            chosen_index = int(raw_input("Choose the object you want to pickup: "))

            print " ---> Picking up the ", userdata.objects_data_in[chosen_index].model_description
            print "----------------------------------"
            print ""
            userdata.graspable_object_out = userdata.graspable_objects_in[chosen_index]
            userdata.graspable_object_name_out = userdata.graspable_objects_names_in[chosen_index]
            userdata.collision_support_surface_name_out = userdata.collision_support_surface_name_in

            print userdata.graspable_objects_names_in[chosen_index]
            print userdata.collision_support_surface_name_in
        except:
            return 'failed'

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

        #initializing the other state machines
        self.object_detection = SrObjectDetectionAndRecognitionStateMachine()
        self.pickup_object = SrPickupObjectStateMachine()
        self.place_object = SrPlaceObjectStateMachine()

        self.introspection_server = smach_ros.IntrospectionServer("FullObjectManipulation", self.state_machine, "/FullObjectManipulation")

        with self.state_machine:
            self.state_machine.add('Starting', Starting(),
                                   transitions={'success':'DetectingAndRecognizingObjects', 'failed':'Finishing'})
            self.state_machine.add('DetectingAndRecognizingObjects', self.object_detection.state_machine,
                                   transitions={'success':'UserChooseObject'} )

            self.state_machine.add('UserChooseObject', UserChooseObject(),
                                   transitions={'success':'PickupObject',
                                                'failed':'DetectingAndRecognizingObjects'},
                                   remapping={'objects_data_in':'objects_data_out',
                                              'graspable_objects_in':'graspable_objects_out',
                                              'graspable_objects_names_in':'graspable_objects_names_out',
                                              'collision_support_surface_name_in':'collision_support_surface_name_out'})

            self.state_machine.add('PickupObject', self.pickup_object.state_machine,
                                   transitions={'success':'PlaceObject', 'failed':'Finishing'},
                                   remapping={'graspable_object_in':'graspable_object_out',
                                              'graspable_object_name_in':'graspable_object_name_out',
                                              'collision_support_surface_name_in':'collision_support_surface_name_out'})

            self.state_machine.add('PlaceObject', self.place_object.state_machine,
                                   transitions={'success': 'Finishing', 'failed':'Finishing'},
                                   remapping={'graspable_object_in':'graspable_object_out',
                                              'graspable_object_name_in':'graspable_object_name_out',
                                              'collision_support_surface_name_in':'collision_support_surface_name_out',
                                              'pickup_result_in':'pickup_result_out'})

            self.state_machine.add('Finishing', Finishing(),
                                   transitions={'restart':'Starting'},
                                   remapping={'objects_data_in':'objects_data_out'})

        self.introspection_server.start()

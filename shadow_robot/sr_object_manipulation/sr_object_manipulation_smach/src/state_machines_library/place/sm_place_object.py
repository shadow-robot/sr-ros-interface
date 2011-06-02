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

from ..sr_generic_state_machine import SrGenericStateMachine
from place_object import PlaceObject, PlacePoses

class SrPlaceObjectStateMachine(SrGenericStateMachine):
    """
    State machine controlling the placement of a selected object.
    """

    def __init__(self):
        """
        """
        SrGenericStateMachine.__init__(self, sm_name="place_object",
                                       sm_input_keys=['graspable_object_in',
                                                      'graspable_object_name_in',
                                                      'collision_support_surface_name_in',
                                                      'pickup_result_in'],
                                       sm_output_keys=['place_result_out'])
        with self.state_machine:
            smach.StateMachine.add('ComputingListOfPoses', ComputingListOfPoses(),
                                   transitions = {'success':'PlacingObject'})

            smach.StateMachine.add('PlacingObject', PlacingObject(),
                                   remapping={'graspable_object_in':'graspable_object_out',
                                              'graspable_object_name_in':'graspable_object_name_out',
                                              'collision_support_surface_name_in':'collision_support_surface_name_out',
                                              'pickup_result_in':'pickup_result_out',
                                              'list_of_poses_in':'list_of_poses_out'})


class ComputingListOfPoses(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'],
                             input_keys=['graspable_object_in',
                                         'graspable_object_name_in',
                                         'collision_support_surface_name_in',
                                         'pickup_result_in'],
                             output_keys=['graspable_object_out',
                                          'graspable_object_name_out',
                                          'collision_support_surface_name_out',
                                          'pickup_result_out',
                                          'list_of_poses_out'])
        self.place_poses = PlacePoses()

    def execute(self, userdata):
        list_of_poses = self.place_poses.compute_list_of_poses()
        if len(list_of_poses) == 0:
            return 'failed'

        userdata.graspable_object_out = userdata.graspable_object_in
        userdata.graspable_object_name_out = userdata.graspable_object_name_in
        userdata.collision_support_surface_name_out = userdata.collision_support_surface_name_in
        userdata.pickup_result_out = userdata.pickup_result_in

        userdata.list_of_poses_out = list_of_poses

        return 'success'


class PlacingObject(smach.State):
    """
    Placing the object we're currently holding at the
    given position.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'],
                             input_keys=['graspable_object_in',
                                         'graspable_object_name_in',
                                         'collision_support_surface_name_in',
                                         'pickup_result_in',
                                         'list_of_poses_in'],
                             output_keys=['place_result_out'])

        self.place_object = PlaceObject()
        self.place_object.activate()

    def execute(self, userdata):
        """
        Place the object.
        """

        print "PLACING THE OBJECT"

        result = self.place_object.execute(graspable_object = userdata.graspable_object_in,
                                           graspable_object_name = userdata.graspable_object_name_in,
                                           collision_support_surface_name = userdata.collision_support_surface_name_in,
                                           pickup_grasp = userdata.pickup_result_in.grasp,
                                           list_of_poses = userdata.list_of_poses_in)

        if result == -1:
            return 'failed'

        userdata.place_result_out = result

        return 'success'

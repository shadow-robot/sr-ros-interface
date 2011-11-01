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
from pickup_object import PickupObject

class SrPickupObjectStateMachine(SrGenericStateMachine):
    """
    This is the state machine controlling the pickup of a selected object.
    """

    def __init__(self, ):
        """
        """
        SrGenericStateMachine.__init__(self, sm_name="object_pickup",
                                       sm_input_keys=['graspable_object_in',
                                                      'graspable_object_name_in',
                                                      'collision_support_surface_name_in'],
                                       sm_output_keys=['pickup_result_out',
                                                       'graspable_object_out',
                                                       'graspable_object_name_out',
                                                       'collision_support_surface_name_out'])
        with self.state_machine:
            smach.StateMachine.add('PickingUpObject', PickingUpObject())

class PickingUpObject(smach.State):
    """
    Picking up one object.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'],
                             input_keys=['graspable_object_in',
                                         'graspable_object_name_in',
                                         'collision_support_surface_name_in'],
                             output_keys=['pickup_result_out',
                                          'graspable_object_out',
                                          'graspable_object_name_out',
                                          'collision_support_surface_name_out'])
        self.object_pickup = PickupObject()
        self.object_pickup.activate()

    def execute(self, userdata):
        """
        Pickup the object
        """
        result = self.object_pickup.execute(graspable_object = userdata.graspable_object_in,
                                            graspable_object_name = userdata.graspable_object_name_in,
                                            collision_support_surface_name = userdata.collision_support_surface_name_in)

        if result == -1:
            return 'failed'

        userdata.pickup_result_out = result
        userdata.graspable_object_out = userdata.graspable_object_in
        userdata.graspable_object_name_out = userdata.graspable_object_name_in
        userdata.collision_support_surface_name_out = userdata.collision_support_surface_name_in

        return 'success'

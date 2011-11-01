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
from object_detection_and_recognition import ObjectDetection, CollisionMapProcessing, ObjectRecognition
from object_detection_and_recognition import ObjectDetectionError, CollisionMapError, ObjectRecognitionError


class SrObjectDetectionAndRecognitionStateMachine(SrGenericStateMachine):
    """
    This is the state machine used to detect and recognize the objects.
    It executes the following actions:
     - detect the table
     - segment the objects on top of the table
     - try to recognize them
    """

    def __init__(self, ):
        """
        Initializes the State Machine for detecting and recognizing the objects.

        The possible outcome for this machine are:
         - success: return a list of found objects
         - failed:  a problem was encountered
        """
        SrGenericStateMachine.__init__(self, sm_name="object_detection",
                                       sm_output_keys=['objects_data_out',
                                                       'graspable_objects_out',
                                                       'graspable_objects_names_out',
                                                       'collision_support_surface_name_out'])

        with self.state_machine:
            smach.StateMachine.add('DetectingObjects', DetectingObjects(),
                                   transitions={'objects_found':'ProcessingCollisionMap',
                                                'no_objects':'DetectingObjects'})

            smach.StateMachine.add('ProcessingCollisionMap', ProcessingCollisionMap(),
                                   transitions={'success':'RecognizingObjects'},
                                   remapping={'objects_data_in':'objects_data_out'})

            smach.StateMachine.add('RecognizingObjects', RecognizingObjects(),
                                   remapping={'collision_map_in':'collision_map_out'})


class DetectingObjects(smach.State):
    """
    Trying to detect the objects above the table.
    """

    def __init__(self):
        """
        Detecting the objects on top of the table.

        No inputs.

        Outputs:
          - objects_data_out contains a table of detected objects

        The possible outcomes for this state are:
         - object_founds: detected the objects, returns a bounding box
         - no_object:     couldn't detect any objects
         - failed:        problem encountered
        """
        smach.State.__init__(self, outcomes=['objects_found', 'no_objects', 'failed'],
                             output_keys=['objects_data_out'])

        self.object_detection = ObjectDetection()
        self.object_detection.activate()

    def execute(self, userdata):
        """
        Starts the detection.
        """
        results = []
        try:
            results = self.object_detection.execute()
        except ObjectDetectionError, e:
            rospy.logerr(e.msg)
            return 'failed'

        #nothing was found
        if results == []:
            return 'no_objects'

        userdata.objects_data_out = results
        return 'objects_found'


class ProcessingCollisionMap(smach.State):
    """
    Processing the collision map.
    """

    def __init__(self):
        """
        Processing the collision map: adding the detected objects, etc...

        The possible outcomes for this state are:
         - success: collision map updated correctly
         - failed:  problem encountered
        """
        smach.State.__init__(self, outcomes=['success', 'failed'],
                             output_keys=['collision_map_out'],
                             input_keys =['objects_data_in'])

        self.collision_map_processor = CollisionMapProcessing()
        self.collision_map_processor.activate()

    def execute(self, userdata):
        """
        Starts the detection.
        """
        results = 0
        try:
            results = self.collision_map_processor.execute(userdata.objects_data_in)
        except CollisionMapError,e:
            rospy.logerr(e.msg)
            return 'failed'

        userdata.collision_map_out = results
        return 'success'


class RecognizingObjects(smach.State):
    """
    Trying to recognize the objects above the table.
    """

    def __init__(self):
        """
        Recognizing the objects on top of the table.

        The possible outcomes for this state are:
         - success: no problems were encountered, may or may not have recognized the objects.
         - failed:  problem encountered
        """
        smach.State.__init__(self, outcomes=['success','failed'],
                             output_keys=['objects_data_out',
                                          'graspable_objects_out',
                                          'graspable_objects_names_out',
                                          'collision_support_surface_name_out'],
                             input_keys =['collision_map_in'])

        self.object_recognition = ObjectRecognition()
        self.object_recognition.activate()

    def execute(self, userdata):
        """
        Starts the detection.
        """
        results = 0

        userdata.collision_support_surface_name_out = userdata.collision_map_in.collision_support_surface_name
        userdata.graspable_objects_out = userdata.collision_map_in.graspable_objects
        userdata.graspable_objects_names_out = userdata.collision_map_in.collision_object_names

        try:
            results = self.object_recognition.execute(userdata.collision_map_in)
        except ObjectRecognitionError, e:
            rospy.logerr(e.msg)
            return 'failed'

        print ""
        print "---"
        for obj in results:
            print obj.model_description
        print "---"

        userdata.objects_data_out = results
        return 'success'





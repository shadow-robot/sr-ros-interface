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


from tabletop_object_detector.srv import TabletopDetection
from tabletop_object_detector.msg import TabletopDetectionResult
from household_objects_database_msgs.srv import GetModelDescription


from sr_generic_state_machine import SrGenericStateMachine

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
                                       sm_output_keys=['objects_data_out'])

        with self.state_machine:
            smach.StateMachine.add('DetectingTable', DetectingTable(),
                                   transitions={'table_found':'DetectingObjects',
                                                'no_table':'DetectingTable'},
                                   remapping={'table_data_in':'table_data_out'})
            
            smach.StateMachine.add('DetectingObjects', DetectingObjects(),
                                   transitions={'objects_found':'RecognizingObjects',
                                                'no_objects':'DetectingObjects'},
                                   remapping={'objects_data_in':'objects_data_out'})
            smach.StateMachine.add('RecognizingObjects', RecognizingObjects())

class DetectingTable(smach.State):
    """
    Trying to detect the table in the point cloud.
    """
    
    def __init__(self):
        """
        Detecting the table.

        The possible outcomes for this state are:
         - table_found: detected the table, returns a bounding box
         - no_table:    couldn't detect any table
         - failed:      problem encountered
        """
        smach.State.__init__(self, outcomes=['table_found', 'no_table', 'failed'],
                             output_keys=['table_data_out'])

    def execute(self, userdata):
        """
        Starts the detection.
        """
        userdata.table_data_out = ['data']
        return 'table_found'

class DetectingObjects(smach.State):
    """
    Trying to detect the objects above the table.
    """
    
    def __init__(self):
        """
        Detecting the objects on top of the table.

        The possible outcomes for this state are:
         - object_founds: detected the objects, returns a bounding box
         - no_object:     couldn't detect any objects
         - failed:        problem encountered
        """
        smach.State.__init__(self, outcomes=['objects_found', 'no_objects', 'failed'],
                             output_keys=['objects_data_out'],
                             input_keys =['table_data_in'])

    def execute(self, userdata):
        """
        Starts the detection.
        """
        userdata.objects_data_out = ['data']
        return 'objects_found'

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
                             output_keys=['objects_data_out'],
                             input_keys =['objects_data_in'])

    def execute(self, userdata):
        """
        Starts the detection.
        """
        userdata.objects_data_out = ['data']
        return 'success'
        




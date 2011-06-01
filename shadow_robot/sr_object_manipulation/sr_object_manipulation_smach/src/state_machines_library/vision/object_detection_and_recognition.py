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


from tabletop_object_detector.srv import TabletopDetection
from tabletop_object_detector.msg import TabletopDetectionResult
from household_objects_database_msgs.srv import GetModelDescription
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing

##########################
#   ERRORS & EXCEPTIONS  #
##########################
class ObjectDetectionAndRecognitionError(Exception):
    """
    Base class for exception regarding the object detection and recognition.
    """
    pass

class ObjectDetectionError(ObjectDetectionAndRecognitionError):
    """
    Exception raised while trying to detect the objects.

    @msg: message describing the error we encountered.
    """
    def __init__(self, msg):
        self.msg = msg

class CollisionMapError(ObjectDetectionAndRecognitionError):
    """
    Exception raised while trying to process the collision map.

    @msg: message describing the error we encountered.
    """
    def __init__(self, msg):
        self.msg = msg

class ObjectRecognitionError(ObjectDetectionAndRecognitionError):
    """
    Exception raised while trying to recognize the objects.

    @msg: message describing the error we encountered.
    """
    def __init__(self, msg):
        self.msg = msg

##################################
#   INTERFACES TO THE ROS NODES  #
##################################
class ObjectDetection(object):
    """
    Class doing the object segmentation. This class
    is instantiated in the SrObjectDetectionAndRecognitionStateMachine.
    """

    def __init__(self):
        """
        The connections to the services are really done in the activate function:
        this way we can wait for the services to appear in a loop in the state
        machine.
        """
        self.service_object_detector = None

    def activate(self):
        """
        Waits for the different services to appear, and initializes the connection
        to the services.
        """
        if self.service_object_detector == None:
            rospy.wait_for_service('object_detection')
            self.service_object_detector = rospy.ServiceProxy('object_detection', TabletopDetection)

    def execute(self):
        """
        Detects the table, and segments the different objects on top of it.

        @raise ObjectDetectionError if a problem was encountered.
        @return list of detected objects
        """
        try:
            detected_objects = self.service_object_detector(True, True, 1)
        except rospy.ServiceException, e:
            raise ObjectDetectionError("Service did not process request: %s" % str(e))

        #returns the detected objects
        return detected_objects


class CollisionMapProcessing(object):
    """
    Class building the collision map. This class
    is instantiated in the SrObjectDetectionAndRecognitionStateMachine.
    """

    def __init__(self):
        """
        The connections to the services are really done in the activate function:
        this way we can wait for the services to appear in a loop in the state
        machine.
        """
        self.service_tabletop_collision_map = None

    def activate(self):
        """
        Waits for the different services to appear, and initializes the connection
        to the services.
        """
        if self.service_tabletop_collision_map == None:
            rospy.wait_for_service('/tabletop_collision_map_processing/tabletop_collision_map_processing')
            self.service_tabletop_collision_map = rospy.ServiceProxy('/tabletop_collision_map_processing/tabletop_collision_map_processing', TabletopCollisionMapProcessing)

    def execute(self, detected_objects, reset_static_map = True, reset_collision_models = True, reset_attached_models = True,
                take_static_collision_map = True, base_link = "/base_link"):
        """
        Detects the table, and segments the different objects on top of it.

        @detected_objects The list of objects detected in the ObjectDetection Class.

        @raise a CollionMapError if a problem was encountered.
        @raise a CollisionMapError if collision map failed to process

        @return teh collision support surface name
        """
        res = 0
        try:
            res = self.service_tabletop_collision_map.call(detected_objects.detection, reset_static_map, reset_collision_models,
                                                           reset_attached_models, take_static_collision_map, base_link)
        except rospy.ServiceException, e:
            raise CollisionMapError("Service did not process request: %s" % str(e))

        if res != 0:
            return res
        else:
            raise CollisionMapError("Failed to process the collision map.")


class ObjectRecognition(object):
    """
    Class doing the object recognition. This class
    is instantiated in the SrObjectDetectionAndRecognitionStateMachine.
    """

    def __init__(self):
        """
        The connections to the services are really done in the activate function:
        this way we can wait for the services to appear in a loop in the state
        machine.
        """
        self.service_db_get_model_description = None
        self.unknown_object_counter = 0

    def activate(self):
        """
        Waits for the different services to appear, and initializes the connection
        to the services.
        """
        if self.service_db_get_model_description == None:
            rospy.wait_for_service('objects_database_node/get_model_description')
            self.service_db_get_model_description = rospy.ServiceProxy('objects_database_node/get_model_description', GetModelDescription)

    def execute(self, collision_map_results):
        """
        Tries to recognize the different objects added to the collision map.

        @collision_map_results The result from the collision map processing. Contains a list of potential models for
        each segmented object.

        @raise ObjectRecognitionError if problem encountered

        @return the list of detected objects updated with the model names and graspable objects
        """
        detected_objects = []

        for grasp_obj, grasp_obj_name in zip(collision_map_results.graspable_objects, collision_map_results.collision_object_names):
            model_index = -1
            obj_tmp = GraspableObject(grasp_obj_name, grasp_obj)
            obj_tmp.graspable_object = grasp_obj
            obj_tmp.graspable_object_name = grasp_obj_name

            if len(grasp_obj.potential_models) > 0:
                model_index = grasp_obj.potential_models[0].model_id

            try:
                obj_tmp.model_description = self.get_object_name(model_index)
            except ObjectRecognitionError, e:
                raise ObjectRecognitionError(e.msg)

            detected_objects.append( obj_tmp )

        return detected_objects

    def get_object_name(self, model_id):
        """
        return the object name given its index (read from database, or
        create unique name if unknown object).
        """
        name = ""

        #todo make sure name is unique
        if model_id == -1:
            name = "unknown_" + str(self.unknown_object_counter)
            self.unknown_object_counter += 1
        else:
            try:
                tmp = self.service_db_get_model_description(model_id)
                name = tmp.name

            except rospy.ServiceException, e:
                raise ObjectRecognitionError("Service did not process request: %s" % str(e) )

        return name


class GraspableObject(object):
    def __init__(self, graspable_object_name, graspable_object):
        self.graspable_object_name = graspable_object_name
        self.graspable_object = graspable_object
        self.model_index = -1

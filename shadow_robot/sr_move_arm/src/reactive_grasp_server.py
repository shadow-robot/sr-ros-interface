#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_move_arm')
import rospy

from object_manipulation_msgs.msg import ReactiveGraspAction, ReactiveGraspResult, \
    ManipulationResult, GripperTranslation, ReactiveLiftAction, ReactiveLiftResult, \
    ReactivePlaceAction, ReactivePlaceResult, ReactiveGraspFeedback, \
    ReactiveLiftFeedback, ReactivePlaceFeedback, ManipulationPhase

class ReactiveGraspActionServer(object):
    def __init__(self, which_arm):
        """
        Class for the reactive grasp actions and services.

        @which_arm is 'r' or 'l'
        """

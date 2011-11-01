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

from geometry_msgs.msg import Vector3Stamped
from object_manipulation_msgs.msg import PickupGoal, PickupAction
import actionlib
from actionlib_msgs.msg import *


##################################
#   INTERFACES TO THE ROS NODES  #
##################################
class PickupObject(object):
    """
    Pickup a given object, this class is instantiated in the SrPickupObjectStateMachine.
    """

    def __init__(self, ):
        """
        The connections to the services are really done in the activate function:
        this way we can wait for the services to appear in a loop in the state
        machine.
        """
        self.pickup_client = None

    def activate(self):
        """
        Waits for the different services to appear, and initializes the connection
        to the services.
        """
        self.pickup_client = actionlib.SimpleActionClient('/object_manipulator/object_manipulator_pickup', PickupAction)
        self.pickup_client.wait_for_server()
        rospy.loginfo("Pickup server ready")

    def execute(self, graspable_object, graspable_object_name,
                collision_support_surface_name,
                pickup_direction=None, arm_name="right_arm",
                desired_approach_distance = 0.05,
                min_approach_distance = 0.02,
                desired_lift_distance = 0.25,
                min_lift_distance = 0.2):
        """
        Pickup the object.
        """
        pickup_goal = PickupGoal()

        pickup_goal.target = graspable_object
        pickup_goal.collision_object_name = graspable_object_name
        pickup_goal.collision_support_surface_name = collision_support_surface_name

        pickup_goal.arm_name = arm_name
        pickup_goal.desired_approach_distance = desired_approach_distance
        pickup_goal.min_approach_distance = min_approach_distance

        if pickup_direction != None:
            pickup_goal.lift.direction = pickup_direction
        else:
            #follow z direction
            pickup_direction = Vector3Stamped()
            pickup_direction.header.stamp = rospy.get_rostime()
            pickup_direction.header.frame_id = "/base_link";
            pickup_direction.vector.x = 0;
            pickup_direction.vector.y = 0;
            pickup_direction.vector.z = 1;

            pickup_goal.lift.direction = pickup_direction;

        #request a vertical lift of 15cm after grasping the object
        pickup_goal.lift.desired_distance = desired_lift_distance
        pickup_goal.lift.min_distance = min_lift_distance
        #do not use tactile-based grasping or tactile-based lift
        pickup_goal.use_reactive_lift = True;
        pickup_goal.use_reactive_execution = True;

        self.pickup_client.send_goal(pickup_goal)
        #timeout after 1sec
        #TODO: change this when using the robot
        self.pickup_client.wait_for_result(timeout=rospy.Duration.from_sec(90.0))
        rospy.loginfo("Got Pickup results")

        pickup_result = self.pickup_client.get_result()

        if self.pickup_client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr("The pickup action has failed: " + str(pickup_result.manipulation_result.value) )
            return -1

        return pickup_result

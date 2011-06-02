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

from geometry_msgs.msg import Vector3Stamped, PoseStamped
from object_manipulation_msgs.msg import PlaceGoal, PlaceAction
import actionlib
from actionlib_msgs.msg import *

from tf import transformations
import tf


class PlacePoses(object):
    """
    Compute a list of possible poses to try.
    """
    def __init__(self, number_of_poses = 20):
        self.number_of_poses = number_of_poses

    def compute_list_of_poses(self):
        list_of_poses = []

        #TODO: change this: should try to place the object in
        # a grid on the table
        initial_pose = PoseStamped()
        initial_pose.header.stamp = rospy.get_rostime()
        initial_pose.header.frame_id = "/base_link"
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.position.z = 0.0
        q=transformations.quaternion_about_axis(0.0, (1,0,0))
        initial_pose.pose.orientation.x = q[0]
        initial_pose.pose.orientation.y = q[1]
        initial_pose.pose.orientation.z = q[2]
        initial_pose.pose.orientation.w = q[3]

        rect_w = 0.1
        rect_h = 0.1
        resolution = 0.02
        current_x = initial_pose.pose.position.x - rect_w
        stop_x    = initial_pose.pose.position.x + rect_w
        current_y = initial_pose.pose.position.y - rect_h
        start_y   = current_y
        stop_y    = initial_pose.pose.position.y + rect_h

        while current_x <= stop_x:
            current_x += resolution
            current_y = start_y
            while current_y <= stop_y:
                current_y += resolution

                current_pose = PoseStamped()
                current_pose.header.stamp = rospy.get_rostime()
                current_pose.header.frame_id = "/base_link"
                current_pose.pose.position.x = current_x
                current_pose.pose.position.y = current_y
                current_pose.pose.position.z = initial_pose.pose.position.z
                current_pose.pose.orientation.x = initial_pose.pose.orientation.x
                current_pose.pose.orientation.y = initial_pose.pose.orientation.y
                current_pose.pose.orientation.z = initial_pose.pose.orientation.z
                current_pose.pose.orientation.w = initial_pose.pose.orientation.w

                list_of_poses.append(current_pose)

        return list_of_poses


##################################
#   INTERFACES TO THE ROS NODES  #
##################################
class PlaceObject(object):
    """
    Places the object we're currently roughly at a given position.
    """

    def __init__(self, ):
        """
        """
        self.place_client = None

    def activate(self):
        """
        Waits for the different services to appear, and initializes the connection
        to the services.
        """
        self.place_client = actionlib.SimpleActionClient('/object_manipulator/object_manipulator_place', PlaceAction)
        self.place_client.wait_for_server()
        rospy.loginfo("Place server ready")


    def execute(self, graspable_object, graspable_object_name,
                pickup_grasp, list_of_poses, collision_support_surface_name,
                arm_name="right_arm", place_direction = None,
                desired_retreat_distance = 0.1, min_retreat_distance = 0.05,
                desired_approach_distance = 0.1, min_approach_distance = 0.05,
                place_padding = 0.01):
        """
        Places the object.
        """
        place_goal = PlaceGoal()

        #place at the prepared location
        place_goal.place_locations = list_of_poses

        place_goal.collision_object_name = graspable_object_name
        place_goal.collision_support_surface_name = collision_support_surface_name

        print "collision support surface name: ",collision_support_surface_name

        #information about which grasp was executed on the object,
        #returned by the pickup action
        place_goal.grasp = pickup_grasp

        #use the right rm to place
        place_goal.arm_name = arm_name
        #padding used when determining if the requested place location
        #would bring the object in collision with the environment
        place_goal.place_padding = place_padding
        #how much the gripper should retreat after placing the object
        place_goal.desired_retreat_distance = desired_retreat_distance
        place_goal.min_retreat_distance = min_retreat_distance

        if place_direction == None:
            #By default we put the object down along the "vertical" direction
            #which is along the z axis in the base_link frame
            direction = Vector3Stamped()
            direction.header.stamp = rospy.get_rostime()
            direction.header.frame_id = "/base_link"
            direction.vector.x = 0
            direction.vector.y = 0
            direction.vector.z = -1
            place_goal.approach.direction = direction
        else:
            place_goal.approach.direction = place_direction

        #request a vertical put down motion of 10cm before placing the object
        place_goal.approach.desired_distance = desired_approach_distance
        place_goal.approach.min_distance = min_approach_distance
        #we are not using tactile based placing
        place_goal.use_reactive_place = False

        self.place_client.send_goal(place_goal)
        #timeout after 1sec
        #TODO: change this when using the robot
        self.place_client.wait_for_result(timeout=rospy.Duration.from_sec(90.0))
        rospy.loginfo("Got Place results")

        place_result = self.place_client.get_result()

        if self.place_client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr("The place action has failed: " + str(place_result.manipulation_result.value) )
            return -1

        return place_result

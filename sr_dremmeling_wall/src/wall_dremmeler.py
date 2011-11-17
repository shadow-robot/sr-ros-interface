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

import roslib; roslib.load_manifest('sr_dremmeling_wall')
import rospy
import actionlib

import denso_msgs.msg
from geometry_msgs.msg import Point, Quaternion, Pose

class WallDremmeler(object):
    """
    """

    def __init__(self, ):
        """
        """
        rospy.init_node('sr_dremmeling_wall')

        self.trajectory_client = actionlib.SimpleActionClient('/denso_arm/trajectory', denso_msgs.msg.TrajectoryAction)
        self.trajectory_client.wait_for_server()

        rospy.spin()

    def run(self):
        """
        """
        #First we get the segmented points.
        segmented_points = []
        print "TODO: get a list of segmented points - call your service"

        #Then we get the normal for the wall
        quaternion = Quaternion()
        print "TODO: retrieve the normal for the wall - call your service"

        #We build a list of poses to send to the hand.
        list_of_poses = self.build_poses( segmented_points, quaternion )

        #now we send this to the arm
        goal = denso_msgs.msg.TrajectoryGoal
        goal.trajectory = list_of_poses

        self.trajectory_client.send_goal( goal )
        self.trajectory_client.wait_for_result()

        rospy.loginfo( "Finished Dremmeling the surface: " + str( self.trajectory_client.get_result() ) )


    def build_poses(self, segmented_points, quaternion ):
        list_of_poses = []

        for point in segmented_points:
            pose = Pose()
            pose.position = point
            pose.orientation = quaternion
            list_of_poses.append( pose )

        return list_of_poses



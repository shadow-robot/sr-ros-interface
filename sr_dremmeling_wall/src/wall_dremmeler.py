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
from geometry_msgs.msg import Pose, Quaternion
from kinect_color_segmentation.srv import WallNormale, SurfaceToDremmel

class WallDremmeler(object):
    """
    """

    def __init__(self, ):
        """
        """
        rospy.init_node('sr_dremmeling_wall')

        self.trajectory_client = actionlib.SimpleActionClient('/denso_arm/trajectory', denso_msgs.msg.TrajectoryAction)
        #self.trajectory_client.wait_for_server()

        self.surface_to_dremmel_server = rospy.ServiceProxy( '/kinect_color_segmentation/segment', SurfaceToDremmel)
        self.wall_orientation_server = rospy.ServiceProxy( '/kinect_color_segmentation/get_wall_normale', WallNormale)

        rospy.spin()

    def run(self):
        """
        """
        #First we get the segmented points.
        rospy.loginfo("segmenting the point cloud")
        segmented_points = []
        try:
            res = self.surface_to_dremmel_server()
            segmented_points = res.points
        except:
            rospy.logerr("Couldn't segment the point cloud")
            return

        #Then we get the normal for the wall
        rospy.loginfo("Getting the wall normale")
        wall_normale = Quaternion()
        wall_link = ""
        try:
            res = self.wall_orientation_server()
            wall_normale = res.normale
            wall_link = res.frame_name
        except:
            rospy.logerr("Couldn't get the wall normale")

        #We build a list of poses to send to the hand.
        list_of_poses = self.build_poses( segmented_points, wall_normale, wall_link )

        #now we send this to the arm
        rospy.loginfo("Sending the list of poses to the arm")
        goal = denso_msgs.msg.TrajectoryGoal
        goal.trajectory = list_of_poses

        self.trajectory_client.send_goal( goal )
        self.trajectory_client.wait_for_result()

        rospy.loginfo( "Finished Dremmeling the surface: " + str( self.trajectory_client.get_result() ) )


    def build_poses(self, segmented_points, quaternion, rotation_link):
        list_of_poses = []

        for point in segmented_points:
            #create a pose above the point (in the wall frame)
            pose_above = Pose()
            pose_above.position = point
            pose_above.position.z += 0.05
            pose_above.orientation = quaternion

            #then create a pose below the point (inside the wall)
            pose_inside = Pose()
            pose_inside.position = point
            pose_inside.position.z -= 0.005
            pose_inside.orientation = quaternion

            #then transform those two poses in the base_link frame
            print "TODO: transform from ", rotation_link, " to /base_link"

            #add those two poses to the list of poses sent to the arm
            list_of_poses.append( pose_above )
            list_of_poses.append( pose_inside )

        return list_of_poses



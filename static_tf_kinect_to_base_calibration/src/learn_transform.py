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

import roslib; roslib.load_manifest('static_tf_kinect_to_base_calibration')
import rospy

from tf import TransformListener, TransformBroadcaster
from geometry_msgs.msg import Pose

class TransformLearner(object):
    def __init__(self):
        rospy.init_node("learn_transform")

        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        self.possible_base_link_poses = []

        self.rate = rospy.Rate(20)

        self.markers = { "/PATTERN_1": Pose( [-0.56297, 0.11, 0.0035],
                                             [0.0, 0.0, 0.0, 1.0] ),
                         "/PATTERN_2": Pose( [-0.45097, 0.11, 0.0035],
                                             [0.0, 0.0, 0.0, 1.0] ),
                         "/PATTERN_3": Pose( [-0.34097, 0.0, 0.0035],
                                             [0.0, 0.0, 0.0, 1.0] ),
                         "/PATTERN_4": Pose( [-0.34097, -0.11, 0.0035],
                                             [0.0, 0.0, 0.0, 1.0] ),
                         "/PATTERN_5": Pose( [-0.45097, -0.11, 0.0035],
                                             [0.0, 0.0, 0.0, 1.0] ),
                         "/PATTERN_6": Pose( [-0.56297, 0.0, 0.0035],
                                             [0.0, 0.0, 0.0, 1.0] )
                        }

        self.run()

    def run(self):
        while not rospy.is_shutdown():
            for index, marker in enumerate( self.markers.items() ):
                pose_tmp = self.publish_and_get_pose( marker[0], marker[1], index )
                if pose_tmp != None:
                    self.possible_base_link_poses.append( pose_tmp )

            if len( self.possible_base_link_poses ) != 0:
                best_guess_base_link_to_camera = self.average_poses()

                print best_guess_base_link_to_camera

                self.tf_broadcaster.sendTransform( (best_guess_base_link_to_camera.position.x, best_guess_base_link_to_camera.position.y,
                                                    best_guess_base_link_to_camera.position.z),
                                                   (best_guess_base_link_to_camera.orientation.x, best_guess_base_link_to_camera.orientation.y,
                                                    best_guess_base_link_to_camera.orientation.z, best_guess_base_link_to_camera.orientation.w),
                                                   rospy.Time.now(),
                                                   "/camera_link",
                                                   "/computed_base_link" )

            self.rate.sleep()

    def average_poses(self):
        """
        Average the list of poses for the different guesses of where the kinect
        is compared to the base_link
        """
        averaged_pose = Pose()
        for pose in self.possible_base_link_poses:
            averaged_pose.position.x += pose.position.x
            averaged_pose.position.y += pose.position.y
            averaged_pose.position.z += pose.position.z

            averaged_pose.orientation.x += pose.orientation.x
            averaged_pose.orientation.y += pose.orientation.y
            averaged_pose.orientation.z += pose.orientation.z
            averaged_pose.orientation.w += pose.orientation.w

        size = len( self.possible_base_link_poses )
        averaged_pose.position.x /= size
        averaged_pose.position.y /= size
        averaged_pose.position.z /= size

        averaged_pose.orientation.x /= size
        averaged_pose.orientation.y /= size
        averaged_pose.orientation.z /= size
        averaged_pose.orientation.w /= size

        return averaged_pose



    def publish_and_get_pose(self, marker_name, transform_marker_to_base_link, index):
        """
        Publishes a transform between the given marker and the base link, then
        listen for the transform from /base_link to /camera_link.

        The transform between the marker and the base link is known.

        """
        trans = None
        rot = None

        #try to get the pose of the given link_name
        # in the base_link frame.
        success = False

        base_link_name = "/base_link_" + str(index)

        for i in range (0, 500):
            self.tf_broadcaster.sendTransform( transform_marker_to_base_link.position ,
                                               transform_marker_to_base_link.orientation,
                                               rospy.Time.now(),
                                               base_link_name,
                                               marker_name )
            try:
                (trans, rot) = self.tf_listener.lookupTransform( base_link_name, '/camera_link',
                                                                 rospy.Time(0) )
                success = True
                break
            except:
                continue

        pose = None

        if success == False:
            return pose

        pose = Pose()
        pose.position.x = trans[0]
        pose.position.y = trans[1]
        pose.position.z = trans[2]
        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]

        return pose


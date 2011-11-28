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
import tf
import time
import math
import denso_msgs.msg
from geometry_msgs.msg import Pose, Quaternion, PoseStamped, Point
from kinect_color_segmentation.srv import WallNormale, SurfaceToDremmel
from tf.transformations import quaternion_from_euler
from rospy.core import rospyerr

from interactive_marker import InteractiveConnectorSelector

class WallDremmeler(object):
    """
    """

    def __init__(self, ):
        """
        """
        rospy.init_node('sr_dremmeling_wall')

        self.trajectory_client = actionlib.SimpleActionClient('/denso_arm/trajectory', denso_msgs.msg.TrajectoryAction)
        #self.trajectory_client.wait_for_server()

        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        self.surface_to_dremmel_server = rospy.ServiceProxy( '/kinect_segmentation/PointSequenceDetection/segment', SurfaceToDremmel)
        self.wall_orientation_server = rospy.ServiceProxy( '/kinect_segmentation/PointSequenceDetection/get_wall_normale', WallNormale)

        self.interactive_markers = InteractiveConnectorSelector(["camera_link"], self.run, "dremmel_wall")


    def run(self, name):
        """
        """
        #First we get the segmented points.
        rospy.loginfo("segmenting the point cloud")
        segmented_points = []
        try:
            res = self.surface_to_dremmel_server()
            segmented_points = res.points
            rospy.loginfo("cloud length:" + str(len(segmented_points)))
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


        wall_normale = self.rotate_normal(segmented_points, wall_normale, wall_link)

        #Substitution of the surface normal for the tooltip normal to test the dremmelling
        rospy.logerr("TODO: remove this. Using tooltip normal.")
        tooltip_pose = self.get_pose("/denso_arm/tooltip", wall_link)
        wall_normale = tooltip_pose.orientation

        #We build a list of poses to send to the hand.
        list_of_poses, list_of_speeds = self.build_poses( segmented_points, wall_normale, wall_link )

        #now we send this to the arm
        rospy.loginfo("Sending the list of poses to the arm")
        goal = denso_msgs.msg.TrajectoryGoal
        goal.trajectory = list_of_poses
        goal.speed = list_of_speeds

        self.trajectory_client.send_goal( goal )
        self.trajectory_client.wait_for_result()

        rospy.loginfo( "Finished Dremmeling the surface: " + str( self.trajectory_client.get_result() ) )

    def rotate_normal(self, segmented_points, quaternion, rotation_link):
        pose = Pose()
        pose.position.x = segmented_points[0].x
        pose.position.y = segmented_points[0].y
        pose.position.z = segmented_points[0].z
        pose.orientation = quaternion

        normal_name = "/surface_normal"

        pose_rotated = Pose()
        #position is the same, we just want to rotate
        pose_rotated.position.x = 0.0
        pose_rotated.position.y = 0.0
        pose_rotated.position.z = 0.0
        rotation_tmp_array = tf.transformations.quaternion_from_euler(0.0, 0.0, 3.14159266)
        pose_rotated.orientation.x = rotation_tmp_array[0]
        pose_rotated.orientation.y = rotation_tmp_array[1]
        pose_rotated.orientation.z = rotation_tmp_array[2]
        pose_rotated.orientation.w = rotation_tmp_array[3]

        rotated_normal_name = "/surface_normal_rotated"

        rate = rospy.Rate(100.0)
        pose_rotated_base = None
        success = False

        for i in range(0,100):
            self.br.sendTransform((pose.position.x, pose.position.y, pose.position.z),
                                  (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w),
                                  rospy.Time.now(), normal_name, rotation_link)

            self.br.sendTransform((pose_rotated.position.x, pose_rotated.position.y, pose_rotated.position.z),
                                  (pose_rotated.orientation.x, pose_rotated.orientation.y, pose_rotated.orientation.z, pose_rotated.orientation.w),
                                  rospy.Time.now(), rotated_normal_name, normal_name)
            rate.sleep()
            try:
                (trans,rot) = self.listener.lookupTransform(rotation_link, rotated_normal_name, rospy.Time())
                pose_rotated_base = PoseStamped()
                pose_rotated_base.pose.position.x = trans[0]
                pose_rotated_base.pose.position.y = trans[1]
                pose_rotated_base.pose.position.z = trans[2]
                pose_rotated_base.pose.orientation.x = rot[0]
                pose_rotated_base.pose.orientation.y = rot[1]
                pose_rotated_base.pose.orientation.z = rot[2]
                pose_rotated_base.pose.orientation.w = rot[3]
                pose_rotated_base.header.frame_id = rotation_link
                pose_rotated_base.header.stamp = rospy.Time.now()
                success = True
                break
            except:
                pass
                #rospy.logerr("Could not transform from " + pose_above_pose_frame.header.frame_id + " to /base_link")

        if success == False:
            rospy.logerr("Could not transform from " + rotated_normal_name + " to " + rotation_link)

        return pose_rotated_base.pose.orientation

    def calculate_distance(self, point1, point2):
        distance = math.sqrt((point1.x - point2.x) * (point1.x - point2.x)
                             + (point1.y - point2.y) * (point1.y - point2.y)
                             + (point1.z - point2.z) * (point1.z - point2.z))
        return distance


    def get_pose(self, link_name, base_link):
        trans = None
        rot = None

        #try to get the pose of the given link_name
        # in the base_link frame.
        for i in range (0, 500):
            try:
                (trans, rot) = self.listener.lookupTransform( base_link, link_name,
                                                              rospy.Time(0) )
                break
            except Exception, e:
                time.sleep(0.01)
                continue

        pose = None
        pose = Pose()
        pose.position.x = trans[0]
        pose.position.y = trans[1]
        pose.position.z = trans[2]
        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]

        return pose

    def build_poses(self, segmented_points, quaternion, rotation_link):
        list_of_poses = []
        list_of_speeds = []
        previous_point = None
        last_orientation_in_base_link = None

        for index,point in enumerate(segmented_points):
            #create a pose above the point (in the wall frame)
            rospy.loginfo("Doing point: " + str(index))

            send_pose_above = False

            #Create a pose above the surface only if it's the first time, or the point is at more than x meters from the previous
            if (previous_point == None):
                previous_point = Point()
                send_pose_above = True
            elif (self.calculate_distance(previous_point, point) > 0.035):
                send_pose_above = True
            else:
                send_pose_above = False

            #Save current point as previous
            previous_point.x = point.x
            previous_point.y = point.y
            previous_point.z = point.z
            
            pose_on_plane = Pose()
            pose_on_plane.position.x = point.x
            pose_on_plane.position.y = point.y
            pose_on_plane.position.z = point.z
            pose_on_plane.orientation = quaternion
            pose_on_plane_name = "/pose_on_plane_" + str(index)

            if send_pose_above == True:
                pose_above = Pose()
                pose_above.position.x = 0.0
                pose_above.position.y = 0.0
                pose_above.position.z = -0.05
                pose_above.orientation = Quaternion(0,0,0,1)
                pose_above_name = "/pose_above_" + str(index)

            #then create a pose below the point (inside the wall)
            pose_inside = Pose()
            pose_inside.position.x = 0.0
            pose_inside.position.y = 0.0
            pose_inside.position.z = 0.022
            pose_inside.orientation = Quaternion(0,0,0,1)
            pose_inside_name = "/pose_inside_"  + str(index)

            #then transform those two poses in the base_link frame
            #print "TODO: transform from ", rotation_link, " to /base_link"


            pose_above_base_frame = None
            pose_inside_base_frame = None
            success = False
            rate = rospy.Rate(100.0)

            if send_pose_above == True:
                for i in range(0,100):
                    self.br.sendTransform((pose_on_plane.position.x, pose_on_plane.position.y, pose_on_plane.position.z),
                                          (pose_on_plane.orientation.x, pose_on_plane.orientation.y, pose_on_plane.orientation.z, pose_on_plane.orientation.w),
                                          rospy.Time.now(), pose_on_plane_name, rotation_link)
                    rate.sleep()
                    self.br.sendTransform((pose_above.position.x, pose_above.position.y, pose_above.position.z),
                                          (pose_above.orientation.x, pose_above.orientation.y, pose_above.orientation.z, pose_above.orientation.w),
                                          rospy.Time.now(), pose_above_name, pose_on_plane_name)
                    rate.sleep()
                    try:
                        (trans,rot) = self.listener.lookupTransform("/base_link", pose_above_name, rospy.Time())
                        pose_above_base_frame = PoseStamped()
                        pose_above_base_frame.pose.position.x = trans[0]
                        pose_above_base_frame.pose.position.y = trans[1]
                        pose_above_base_frame.pose.position.z = trans[2]
                        pose_above_base_frame.pose.orientation.x = rot[0]
                        pose_above_base_frame.pose.orientation.y = rot[1]
                        pose_above_base_frame.pose.orientation.z = rot[2]
                        pose_above_base_frame.pose.orientation.w = rot[3]
                        pose_above_base_frame.header.frame_id = "/base_link"
                        pose_above_base_frame.header.stamp = rospy.Time.now()
                        success = True
                        break
                    except:
                        pass
                        #rospy.logerr("Could not transform from " + pose_above_pose_frame.header.frame_id + " to /base_link")


                if success == False:
                    rospy.logerr("Could not transform from " + pose_above_name + " to /base_link")

            success = False
            for i in range(0,100):
                self.br.sendTransform((pose_on_plane.position.x, pose_on_plane.position.y, pose_on_plane.position.z),
                                      (pose_on_plane.orientation.x, pose_on_plane.orientation.y, pose_on_plane.orientation.z, pose_on_plane.orientation.w),
                                      rospy.Time.now(), pose_on_plane_name, rotation_link)
                rate.sleep()
                self.br.sendTransform((pose_inside.position.x, pose_inside.position.y, pose_inside.position.z),
                                      (pose_inside.orientation.x, pose_inside.orientation.y, pose_inside.orientation.z, pose_inside.orientation.w),
                                      rospy.Time.now(), pose_inside_name, pose_on_plane_name)
                rate.sleep()

                try:
                    (trans,rot) = self.listener.lookupTransform("/base_link", pose_inside_name, rospy.Time())
                    pose_inside_base_frame = PoseStamped()
                    pose_inside_base_frame.pose.position.x = trans[0]
                    pose_inside_base_frame.pose.position.y = trans[1]
                    pose_inside_base_frame.pose.position.z = trans[2]
                    pose_inside_base_frame.pose.orientation.x = rot[0]
                    pose_inside_base_frame.pose.orientation.y = rot[1]
                    pose_inside_base_frame.pose.orientation.z = rot[2]
                    pose_inside_base_frame.pose.orientation.w = rot[3]
                    pose_inside_base_frame.header.frame_id = "/base_link"
                    pose_inside_base_frame.header.stamp = rospy.Time.now()
                    success = True

                    #Copy orientaion
                    last_orientation_in_base_link = pose_inside_base_frame.pose.orientation
                    break
                except:
                    pass
                    #rospy.logerr("Could not transform from " + pose_inside_pose_frame.header.frame_id + " to /base_link")
                rate.sleep()

            if success == False:
                rospy.logerr("Could not transform from " + pose_inside_name + " to /base_link")


            #add those two poses to the list of poses sent to the arm
            if send_pose_above == True:
                if (pose_above_base_frame != None):
                    list_of_poses.append( pose_above_base_frame.pose )
                    list_of_speeds.append( 30.0 )
                else:
                    rospy.logerr("Could not convert pose above " + pose_above_name + " to /base_link")

            if (pose_inside_base_frame != None):
                list_of_poses.append( pose_inside_base_frame.pose )
                list_of_speeds.append( 20.0 )
            else:
                rospy.logerr("Could not convert pose inside " + pose_inside_name + " to /base_link")

        #Enqueue a last position far from the target
        pose_out = Pose()
        pose_out.position.x = 0.643
        pose_out.position.y = -0.001
        pose_out.position.z = 0.356
        pose_out.orientation = last_orientation_in_base_link
        list_of_poses.append( pose_out )
        list_of_speeds.append( 25.0 )

        return list_of_poses, list_of_speeds

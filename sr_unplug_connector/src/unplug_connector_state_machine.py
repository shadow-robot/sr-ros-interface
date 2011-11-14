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

import roslib; roslib.load_manifest('sr_unplug_connector')
import rospy

import smach, smach_ros, tf, actionlib

import time, numpy

from object_manipulation_msgs.msg import GraspableObject, GraspHandPostureExecutionAction, GraspHandPostureExecutionGoal, ManipulationResult

from object_manipulation_msgs.srv import GraspPlanning
from sensor_msgs.msg import PointCloud
from denso_msgs.msg import MoveArmPoseGoal, MoveArmPoseResult, MoveArmPoseAction
from geometry_msgs.msg import Pose
from re_kinect_object_detector.msg import DetectionResult
from std_srvs.srv import Empty

from interactive_marker import InteractiveConnectorSelector

MAX_LIFT_DISTANCE = 0.2

class UnplugConnectorStateMachine(object):
    """
    """

    def __init__(self, ):
        """
        """
        rospy.init_node("unplug_connector")

        self.running = False
        self.recognition_header = None
        self.detection_subscriber = None
        self.grasp_planner_srv = None
        self.detected_objects = {}

        self.tf_listener = tf.TransformListener()

        self.detection_subscriber = rospy.Subscriber("~input", DetectionResult, self.detection_callback)


        self.pub_pcl = rospy.Publisher("/test", PointCloud)

        rospy.wait_for_service('/sr_grasp_planner/plan_point_cluster_grasp')
        self.grasp_planner_srv = rospy.ServiceProxy( '/sr_grasp_planner/plan_point_cluster_grasp', GraspPlanning )

        self.start_service = rospy.Service('~start', Empty, self.run)

        self.grasp_client = actionlib.SimpleActionClient('/right_arm/hand_posture_execution', GraspHandPostureExecutionAction)
        self.grasp_client.wait_for_server()

        self.denso_arm_client = actionlib.SimpleActionClient('/denso_arm/move_arm_pose', MoveArmPoseAction)
        self.denso_arm_client.wait_for_server()

        rospy.spin()

    def run(self, req):
        if not self.running:
            self.running = True
            list_of_grasps = self.plan_grasp( req )

            if list_of_grasps != []:
                #we received a list of grasps. Select the best one
                best_grasp = self.select_best_grasp( list_of_grasps )

                result = self.grasp_connector( best_grasp )

                if result:
                    self.unplug_connector( best_grasp )
            self.running = False

        return []

    def detection_callback(self, msg):
        self.recognition_header = msg.Image.header

        for index, name in enumerate(msg.ObjectNames):
            self.detected_objects[ name ] = msg.Detections[index]


        self.interactive_markers = InteractiveConnectorSelector(msg.ObjectNames, self.run)

    def plan_grasp(self, name):
        """
        Plan the grasps.

        @return a list containing all the possible grasps for the selected object.
        """
        while self.detected_objects == {}:
            rospy.loginfo("Waiting to identify the object")
            time.sleep(0.1)

        graspable_object = GraspableObject()

        graspable_object.cluster = self.points3d_to_pcl( self.detected_objects[name].points3d )

        rospy.loginfo("Detected object, trying to grasp it")

        try:
            resp1 = self.grasp_planner_srv( arm_name="", target=graspable_object, collision_object_name="",
                                            collision_support_surface_name="", grasps_to_evaluate = [])
        except rospy.ServiceException, e:
            rospy.logerr( "Service did not process request: %s"%str(e) )
            return []

        return resp1.grasps

    def select_best_grasp(self, list_of_grasps):
        best_grasp = list_of_grasps[0]

        #select the best grasp based on the distance from the palm
        # First, we read the pose for the palm.
        palm_pose = self.get_pose("/srh/position/palm")
        #rospy.logdebug( " PALM: ", palm_pose )

        #Get the closest grasp.
        min_distance = self.distance(list_of_grasps[0].grasp_pose, palm_pose)

        tmp_index = 0
        for index, grasp in enumerate(list_of_grasps):
            grasp_pose = grasp.grasp_pose
            distance = self.distance(grasp_pose, palm_pose)
            if distance < min_distance:
                best_grasp = grasp
                min_distance = distance
                tmp_index = index

        #rospy.logdebug( "----" )
        #rospy.logdebug( "BEST GRASP:" )
        #rospy.logdebug( " distance = ", min_distance, " (", tmp_index,")" )
        #rospy.logdebug( "", best_grasp )
        #rospy.logdebug( "----" )
        #then transform the grasp pose to be in the denso arm tf frame
        #may be not necessary: base link is probably the base of the
        #denso arm

        return best_grasp

    def get_pose(self, link_name):
        trans = None
        rot = None

        for i in range (0, 500):
            try:
                (trans, rot) = self.tf_listener.lookupTransform( '/base_link', link_name,
                                                                 rospy.Time(0) )
                break
            except (tf.LookupException, tf.ConnectivityException):
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


    def distance(self, pose1, pose2):
        pose_1_vec = numpy.array( [ pose1.position.x, pose1.position.y, pose1.position.z, pose1.orientation.x,
                                    pose1.orientation.y, pose1.orientation.z, pose1.orientation.w ] )
        pose_2_vec = numpy.array( [ pose2.position.x, pose2.position.y, pose2.position.z, pose2.orientation.x,
                                    pose2.orientation.y, pose2.orientation.z, pose2.orientation.w ] )
        return numpy.linalg.norm( pose_1_vec - pose_2_vec )


    def grasp_connector(self, grasp):
        #First we set the hand to the pregrasp position
        goal = GraspHandPostureExecutionGoal()
        goal.grasp = grasp
        goal.goal = goal.PRE_GRASP

        self.grasp_client.send_goal( goal )
        self.grasp_client.wait_for_result()

        res = self.grasp_client.get_result()
        if res.result.value != ManipulationResult.SUCCESS:
            rospy.logerr("Failed to go to Pregrasp")
            return False

        time.sleep(1)

        #then we move the arm to the grasp pose
        arm_goal = MoveArmPoseGoal()
        arm_goal.goal = grasp.grasp_pose
        arm_goal.rate = 100
        arm_goal.time_out = rospy.Duration.from_sec(2.)

        self.denso_arm_client.send_goal( arm_goal )
        self.denso_arm_client.wait_for_result()

        res =  self.denso_arm_client.get_result()
        if res.val != MoveArmPoseResult.SUCCESS:
            rospy.logerr("Failed to move the arm to the given position.")
            return False

        #finally we grasp the object
        rospy.loginfo("Going to grasp")
        goal.goal = goal.GRASP

        self.grasp_client.send_goal( goal )
        self.grasp_client.wait_for_result()

        res = self.grasp_client.get_result()
        if res.result.value != ManipulationResult.SUCCESS:
            rospy.logerr("Failed to go to Grasp")
            return False

        return True

    def unplug_connector(self, grasp):
        #We compute a list of poses to send to
        # the arm (going up from grasp)
        list_poses = []
        z = 0.0
        lift_step = 0.01
        pose_tmp = grasp.grasp_pose
        while z <= MAX_LIFT_DISTANCE:
            z += lift_step
            pose_tmp.position.z += lift_step
            list_poses.append( pose_tmp )

        # We send one position after the other
        # to the arm.
        for pose_tmp in list_poses:
            arm_goal = MoveArmPoseGoal()
            arm_goal.goal = pose_tmp
            arm_goal.rate = 100
            arm_goal.time_out = rospy.Duration.from_sec(2.)

            self.denso_arm_client.send_goal( arm_goal )
            self.denso_arm_client.wait_for_result()

            res =  self.denso_arm_client.get_result()
            if res.val != MoveArmPoseResult.SUCCESS:
                rospy.logerr("Failed to move the arm to the given position.")
                return False


        #TODO: delete this sleep
        time.sleep(1)

        #finally we release the object
        goal = GraspHandPostureExecutionGoal()
        goal.grasp = grasp
        rospy.loginfo("Going to release the object")
        goal.goal = goal.RELEASE

        self.grasp_client.send_goal( goal )
        self.grasp_client.wait_for_result()

        res = self.grasp_client.get_result()
        if res.result.value != ManipulationResult.SUCCESS:
            rospy.logerr("Failed to go to Release")
            return False

        return True

    def points3d_to_pcl(self, points3d):
        pcl = PointCloud()

        #transforms the points3d list into a pointcloud
        pcl.header = self.recognition_header

        pcl.points = points3d

        self.pub_pcl.publish(pcl)

        return pcl

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
from denso_msgs.msg import MoveArmPoseGoal, MoveArmPoseResult, MoveArmPoseAction, TrajectoryAction, TrajectoryGoal, TrajectoryResult
from geometry_msgs.msg import Pose
from re_kinect_object_detector.msg import DetectionResult
from std_srvs.srv import Empty

from interactive_markers.interactive_marker_server import *
from interactive_marker import InteractiveConnectorSelector
from visualization_msgs.msg import Marker, MarkerArray

class UnplugConnectorStateMachine(object):
    """
    Controls the process of unplugging connector:
    
    Put the hand and the arm in their initial positions (the hand is in a the grasping pose)
    wait for the user to click on a recognised object (the connector we want to unplug)
    calculate grasps around the selected object (selected by the user clicking the interactive marker),
    send the arm to the best grasping position (with some intermediate positions to ensure that no forbidden poses will be asked to the arm),
    grab the object with the hand
    lift the arm (unplugging the connector)
    move the arm to a position above the box where we want to drop the connector
    move the arm down, closer to the box
    open the hand to drop the connector
    move arm to the final position
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
        self.ran_once = False

        self.interactive_marker_server = InteractiveMarkerServer("select_connector")

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()

        self.detection_subscriber = rospy.Subscriber("~input", DetectionResult, self.detection_callback)

        self.publisher_marker_best_pose = rospy.Publisher("~best_pose", MarkerArray)

        rospy.wait_for_service('/sr_grasp_planner/plan_point_cluster_grasp')
        self.grasp_planner_srv = rospy.ServiceProxy( '/sr_grasp_planner/plan_point_cluster_grasp', GraspPlanning )

        self.start_service = rospy.Service('~start', Empty, self.run)

        self.grasp_client = actionlib.SimpleActionClient('/right_arm/hand_posture_execution', GraspHandPostureExecutionAction)
        self.grasp_client.wait_for_server()

        self.denso_trajectory_client = actionlib.SimpleActionClient('/denso_arm/trajectory', TrajectoryAction)
        self.denso_trajectory_client.wait_for_server()

        self.go_to_initial_position()

        rospy.spin()

    def run(self, req):
        if not self.ran_once:
            self.ran_once = True
            if not self.running:
                self.running = True

                list_of_grasps = self.plan_grasp( req )

                if list_of_grasps != []:
                    #we received a list of grasps. Select the best one
                    best_grasp = self.select_best_grasp( list_of_grasps )

                    #grasp using the selected grasp
                    result = self.grasp_connector( best_grasp )

                    if result:
                        #unplug the grasped connector
                        self.unplug_connector( best_grasp )
                        
                        #Go to the final pose
                        self.go_to_final_position()
                    self.running = False
                    

        return []

    def detection_callback(self, msg):
        #called each time an object is detected.
        #update the header and the detected_objects
        self.recognition_header = msg.Image.header

        for index, name in enumerate(msg.ObjectNames):
            self.detected_objects[ name ] = msg.Detections[index]

        self.interactive_markers = InteractiveConnectorSelector(msg.ObjectNames, self.run, self.interactive_marker_server)

    def plan_grasp(self, name):
        """
        Plan the grasps.

        @name name of the selected object
        @return a list containing all the possible grasps for the selected object.
        """
        while self.detected_objects == {}:
            rospy.loginfo("Waiting to identify the object")
            time.sleep(0.1)

        #builds the graspable object to send to the grasp planner from the detected_objects,
        # we compute the grasps for the selected object only
        graspable_object = GraspableObject()
        graspable_object.cluster = self.points3d_to_pcl( self.detected_objects[name].points3d )

        rospy.loginfo("Detected object: "+ name + ", trying to grasp it")

        try:
            resp1 = self.grasp_planner_srv( arm_name="", target=graspable_object, collision_object_name="",
                                            collision_support_surface_name="", grasps_to_evaluate = [])
        except rospy.ServiceException, e:
            rospy.logerr( "Service did not process request: %s"%str(e) )
            return []

        return resp1.grasps

    def select_best_grasp(self, list_of_grasps):
        best_grasp = list_of_grasps[0]

        self.go_to_middle_position()
        #select the best grasp based on the distance from the palm
        # First, we read the pose for the palm.
        palm_pose = self.get_pose("/srh/position/palm")

        #rospy.logdebug( " PALM: ", palm_pose )

        #Get the closest grasp.
        min_distance = self.distance(list_of_grasps[0].grasp_pose, palm_pose)
        tmp_index = 0

        for index, grasp in enumerate(list_of_grasps):
            if grasp.grasp_pose.position.x == 0.0 and grasp.grasp_pose.position.y == 0.0 and grasp.grasp_pose.position.z == 0.0:
                #ignore bad grasps
                continue

            grasp_pose = grasp.grasp_pose
            distance = self.distance(grasp_pose, palm_pose)
            if distance < min_distance:
                best_grasp = grasp
                min_distance = distance
                tmp_index = index

        #pose_tip = self.get_pose("/denso_arm/tooltip")
        #best_grasp.grasp_pose.orientation = pose_tip.orientation
        #print "----"
        #print "BEST GRASP:"
        #print " distance = ", min_distance, " (", tmp_index,")"
        #print "", best_grasp
        #print "----"
        #then transform the grasp pose to be in the denso arm tf frame
        #may be not necessary: base link is probably the base of the
        #denso arm
        for i in range(0,100):
            self.tf_broadcaster.sendTransform( ( best_grasp.grasp_pose.position.x, best_grasp.grasp_pose.position.y, best_grasp.grasp_pose.position.z) ,
                                               ( best_grasp.grasp_pose.orientation.x, best_grasp.grasp_pose.orientation.y,
                                                 best_grasp.grasp_pose.orientation.z, best_grasp.grasp_pose.orientation.w),
                                               rospy.Time.now(),
                                               "selected_pose",
                                               "base_link")

            self.tf_broadcaster.sendTransform( (0.0, 0.0, 0.0),
                                               tf.transformations.quaternion_from_euler(0, -1.57, 0),
                                               rospy.Time.now(),
                                               "selected_pose_rotated",
                                               "selected_pose")
            time.sleep(0.001)

            self.tf_broadcaster.sendTransform( (0.05, 0.115, -0.17),
                                               tf.transformations.quaternion_from_euler(0.17, 0, 0),
                                               rospy.Time.now(),
                                               "selected_pose_for_arm_tip",
                                               "selected_pose_rotated")
            time.sleep(0.001)

        selected_pose_for_arm_tip = self.get_pose("selected_pose_for_arm_tip")
        best_grasp.grasp_pose.position = selected_pose_for_arm_tip.position
        best_grasp.grasp_pose.orientation = selected_pose_for_arm_tip.orientation

        #display the best grasp in rviz
        markerArray = MarkerArray()
        marker_X = Marker()
        marker_X.header.frame_id = "/base_link"
        marker_X.type = marker_X.SPHERE
        marker_X.action = marker_X.ADD
        marker_X.scale.x = 0.02
        marker_X.scale.y = 0.02
        marker_X.scale.z = 0.02
        marker_X.color.a = 1.0
        marker_X.color.r = 1.0
        marker_X.color.g = 0.0
        marker_X.color.b = 0.0
        marker_X.pose.orientation.w = 1.0
        marker_X.pose.position.x = best_grasp.grasp_pose.position.x
        marker_X.pose.position.y = best_grasp.grasp_pose.position.y
        marker_X.pose.position.z = best_grasp.grasp_pose.position.z
        markerArray.markers.append(marker_X)

        # Publish the MarkerArray
        self.publisher_marker_best_pose.publish(markerArray)

        return best_grasp

    def get_pose(self, link_name):
        trans = None
        rot = None

        #try to get the pose of the given link_name
        # in the base_link frame.
        for i in range (0, 500):
            try:
                (trans, rot) = self.tf_listener.lookupTransform( '/base_link', link_name,
                                                                 rospy.Time(0) )
                break
            except :
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
        #compute the distance between two poses.
        # is this the correct method to compute the distance?
        pose_1_vec = numpy.array( [ pose1.position.x, pose1.position.y, pose1.position.z ] )
        pose_2_vec = numpy.array( [ pose2.position.x, pose2.position.y, pose2.position.z ] )
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

        self.go_to_middle_2_position()

        #then we move the arm to the grasp pose
        goal = TrajectoryGoal()
        traj = []
        speed = []
        traj.append( grasp.grasp_pose )
        speed.append( 45. )
        goal.trajectory = traj
        goal.speed = speed
        
        self.denso_trajectory_client.send_goal( goal )
        self.denso_trajectory_client.wait_for_result()
        res =  self.denso_trajectory_client.get_result()
        if res.val != TrajectoryResult.SUCCESS:
            rospy.logerr("Failed to move the arm to the given position.")
            return False

        #finally we grasp the object
        goal = GraspHandPostureExecutionGoal()
        goal.grasp = grasp
        goal.goal = goal.PRE_GRASP

        rospy.loginfo("Going to grasp")
        goal.goal = goal.GRASP

        self.grasp_client.send_goal( goal )
        self.grasp_client.wait_for_result()

        res = self.grasp_client.get_result()
        if res.result.value != ManipulationResult.SUCCESS:
            rospy.logerr("Failed to go to Grasp")
            return False
        rospy.loginfo("OK grasped")

        return True

    def unplug_connector(self, grasp):
        #We compute a list of poses to send to
        # the arm (going up from grasp)

        rospy.loginfo("unplugging the connector")

        goal = TrajectoryGoal()
        traj = []
        speed = []
        pose_tmp = grasp.grasp_pose
        pose_tmp.position.z += 0.1
        traj.append( pose_tmp )
        speed.append( 30. )
        goal.trajectory = traj
        goal.speed = speed

        self.denso_trajectory_client.send_goal( goal )
        self.denso_trajectory_client.wait_for_result()
        res =  self.denso_trajectory_client.get_result()
        if res.val != TrajectoryResult.SUCCESS:
            rospy.logerr("Failed to move the arm to the given position.")
            return False

        rospy.loginfo("unplugged the connector")

	#Move the arm to a fixed position (above the box where the connector will be dropped)
        goal = TrajectoryGoal()
        traj = []
        speed = []
        pose_above_box = Pose()
        pose_above_box.position.x = 0.454051
        pose_above_box.position.y = 0.283831
        pose_above_box.position.z = 0.296853
        pose_above_box.orientation.x = 0.711336
        pose_above_box.orientation.y = -0.31226
        pose_above_box.orientation.z = 0.577
        pose_above_box.orientation.w = 0.25212
        traj.append( pose_above_box )
        speed.append( 100. )

        #pose_in_box = Pose()
        #pose_in_box.position.x = 0.561363
        #pose_in_box.position.y = 0.229195
        #pose_in_box.position.z = 0.228286
        #pose_in_box.orientation.x = 0.772113
        #pose_in_box.orientation.y = -0.318396
        #pose_in_box.orientation.z = 0.532802
        #pose_in_box.orientation.w = 0.198744
        #traj.append( pose_in_box )
        #speed.append( 10. )
        
        goal.trajectory = traj
        goal.speed = speed

        rospy.loginfo("Putting the connector in the box")
        
        self.denso_trajectory_client.send_goal( goal )
        self.denso_trajectory_client.wait_for_result()
        res =  self.denso_trajectory_client.get_result()
        if res.val != TrajectoryResult.SUCCESS:
            rospy.logerr("Failed to move the arm to the given position.")
            return False
        
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

        time.sleep(0.1)

        rospy.loginfo("Released the connector")
        
        return True

    def points3d_to_pcl(self, points3d):
        pcl = PointCloud()

        #transforms the points3d list into a pointcloud
        pcl.header = self.recognition_header

        pcl.points = points3d
        return pcl

    def go_to_initial_position(self):
        #Move the arm to a fixed initial position
        goal = TrajectoryGoal()
        traj = []
        speed = []
        pose_tmp = Pose()
        pose_tmp.position.x = 0.157454
        pose_tmp.position.y = 0.536324
        pose_tmp.position.z = 0.357698
        pose_tmp.orientation.x = 0.736802
        pose_tmp.orientation.y = 0.147973
        pose_tmp.orientation.z = 0.500246
        pose_tmp.orientation.w = 0.430094

        traj.append( pose_tmp )
        speed.append( 100. )

        goal.trajectory = traj
        goal.speed = speed

        self.denso_trajectory_client.send_goal( goal )
        self.denso_trajectory_client.wait_for_result()
        res =  self.denso_trajectory_client.get_result()
        if res.val != TrajectoryResult.SUCCESS:
            rospy.logerr("Failed to move the arm to the given position.")
            return False

        return True


    def go_to_middle_position(self):
        #Move the arm to a fixed middle position
        goal = TrajectoryGoal()
        traj = []
        speed = []
        pose_tmp = Pose()
        pose_tmp.position.x = 0.209194
        pose_tmp.position.y = 0.493818
        pose_tmp.position.z = 0.301134
        pose_tmp.orientation.x = 0.700753
        pose_tmp.orientation.y = -0.254614
        pose_tmp.orientation.z = 0.652508
        pose_tmp.orientation.w = 0.135466

        traj.append( pose_tmp )
        speed.append( 100. )

        goal.trajectory = traj
        goal.speed = speed

        self.denso_trajectory_client.send_goal( goal )
        self.denso_trajectory_client.wait_for_result()
        res =  self.denso_trajectory_client.get_result()
        if res.val != TrajectoryResult.SUCCESS:
            rospy.logerr("Failed to move the arm to the given position.")
            return False
        return True

    def go_to_middle_2_position(self):
        #Move the arm to a fixed middle position
        goal = TrajectoryGoal()
        traj = []
        speed = []
        pose_tmp = Pose()
        pose_tmp.position.x = 0.217902
        pose_tmp.position.y = 0.213502
        pose_tmp.position.z = 0.339546
        pose_tmp.orientation.x = 0.700753
        pose_tmp.orientation.y = -0.504614
        pose_tmp.orientation.z = 0.702508
        pose_tmp.orientation.w = 0.0511

        traj.append( pose_tmp )
        speed.append( 100. )

        goal.trajectory = traj
        goal.speed = speed

        self.denso_trajectory_client.send_goal( goal )
        self.denso_trajectory_client.wait_for_result()
        res =  self.denso_trajectory_client.get_result()
        if res.val != TrajectoryResult.SUCCESS:
            rospy.logerr("Failed to move the arm to the given position.")
            return False

        return True


    def go_to_final_position(self):
        #Move the arm to a fixed final position
        goal = TrajectoryGoal()
        traj = []
        speed = []
        pose_tmp = Pose()
        pose_tmp.position.x = 0.138631
        pose_tmp.position.y = 0.433044
        pose_tmp.position.z = 0.44185
        pose_tmp.orientation.x = 0.700738
        pose_tmp.orientation.y = -0.254597
        pose_tmp.orientation.z = 0.652538
        pose_tmp.orientation.w = 0.135428

        traj.append( pose_tmp )
        speed.append( 100. )

        goal.trajectory = traj
        goal.speed = speed

        self.denso_trajectory_client.send_goal( goal )
        self.denso_trajectory_client.wait_for_result()
        res =  self.denso_trajectory_client.get_result()
        if res.val != TrajectoryResult.SUCCESS:
            rospy.logerr("Failed to move the arm to the given position.")
            return False

        return True

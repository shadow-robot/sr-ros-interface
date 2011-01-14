#! /usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# author: Kaijen Hsiao

## @package collision_map_interface
# Python version of collision_map_interface.cpp
# Functions that take and reset the collision map, and add, remove, attach
# or detach objects

import roslib; roslib.load_manifest('tabletop_collision_map_processing')
import random, time, math, scipy, pdb
import rospy
from geometric_shapes_msgs.msg import Shape
from tabletop_object_detector.msg import Table
from collision_environment_msgs.msg import MakeStaticCollisionMapAction, MakeStaticCollisionMapGoal
from mapping_msgs.msg import CollisionObject, AttachedCollisionObject, CollisionObjectOperation
from actionlib_msgs.msg import GoalStatus
from std_srvs.srv import Empty, EmptyRequest
from geometric_shapes_msgs.msg import Shape
import actionlib
import tf
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose

#interface to the collision map
class CollisionMapInterface():

    def __init__(self):

        #set up service clients, publishers, and action clients
        rospy.loginfo("waiting for collision_map_self_occ_node/reset service")
        rospy.wait_for_service("collision_map_self_occ_node/reset")
        self.collision_map_reset_client = rospy.ServiceProxy("collision_map_self_occ_node/reset", Empty)
        
        self.make_static_collision_map_client = actionlib.SimpleActionClient('make_static_collision_map', MakeStaticCollisionMapAction)
        rospy.loginfo("waiting for make_static_collision_map action server")
        self.make_static_collision_map_client.wait_for_server()
        
        rospy.loginfo("advertising on collision object topics")
        self.object_in_map_pub = rospy.Publisher("collision_object", CollisionObject)
        self.attached_object_pub = rospy.Publisher("attached_collision_object", AttachedCollisionObject)

        self.object_in_map_current_id = 0
        rospy.loginfo("done initializing collision map interface")


    ##reset the current collision map to default
    def reset_collision_map(self):

        #reset the collision matrix
        try:
            resp = self.collision_map_reset_client(EmptyRequest())
        except rospy.ServiceException, e:
            rospy.logerr("error when calling collision_map_self_occ_node/reset: %s"%e)
            return 0

        #remove all objects
        reset_object = CollisionObject()
        reset_object.operation.operation = CollisionObjectOperation.REMOVE
        reset_object.header.frame_id = "base_link"
        reset_object.header.stamp = rospy.Time.now()
        reset_object.id = "all"
        self.object_in_map_pub.publish(reset_object)

        #and all attached objects
        reset_attached_objects = AttachedCollisionObject()
        reset_attached_objects.link_name = "all"
        reset_attached_objects.object.header.frame_id = "base_link"
        reset_attached_objects.object.header.stamp = rospy.Time.now()
        reset_attached_objects.object = reset_object
        self.attached_object_pub.publish(reset_attached_objects)

        rospy.loginfo("collision map reset")
        self.object_in_map_current_id = 0.
        return 1


    ##take a static map scan
    def take_static_map(self):
        static_map_goal = MakeStaticCollisionMapGoal()
        static_map_goal.cloud_source = "full_cloud_filtered"
        static_map_goal.number_of_clouds = 2;
        self.make_static_collision_map_client.send_goal(static_map_goal)

        if not self.make_static_collision_map_client.wait_for_result(rospy.Duration(30)):
            rospy.loginfo("collision map was not formed in allowed time")
            return 0

        if self.make_static_collision_map_client.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("static map successfully updated")
            return 1
        else:
            rospy.loginfo("some other non-success state was reached for static collision map.  Proceed with caution.")
            return 0


    ##attaches an object to the gripper for the purposes of collision-aware 
    #motion planning
    def attach_object_to_gripper(self, object_name, whicharm = 'r'):

        obj = AttachedCollisionObject()
        
        obj.link_name = whicharm+"_gripper_r_finger_tip_link"
        obj.object.operation.operation = CollisionObjectOperation.ATTACH_AND_REMOVE_AS_OBJECT
        obj.object.header.stamp = rospy.Time.now()
        obj.object.header.frame_id = "base_link"
        obj.object.id = object_name
        touch_links = ["_gripper_palm_link", "_gripper_r_finger_tip_link", "_gripper_l_finger_tip_link", "_gripper_l_finger_link", "_gripper_r_finger_link"]
        obj.touch_links = [whicharm+link for link in touch_links]
        self.attached_object_pub.publish(obj)


    ##detaches all objects from the gripper 
    #(removes from collision space entirely)
    def detach_all_objects_from_gripper(self, whicharm = 'r'):

        rospy.loginfo("detaching all objects from gripper %s"%whicharm)
        obj = AttachedCollisionObject()
        obj.object.header.stamp = rospy.Time.now()
        obj.object.header.frame_id = "base_link"
        obj.link_name = whicharm+"_gripper_r_finger_tip_link"
        obj.object.operation.operation = CollisionObjectOperation.REMOVE
        self.attached_object_pub.publish(obj)
        

    ##detaches all objects from the gripper 
    #(adds back as objects in the collision space where they are now)
    #object_collision_name is the original name for the collision object
    def detach_and_add_back_objects_attached_to_gripper(self, whicharm = 'r', object_collision_name = None):

        rospy.loginfo("detaching all objects from gripper %s and adding them back to the collision map"%whicharm)

        if object_collision_name == None:
            rospy.loginfo("need to specify the object name!  Detaching and not adding back object.")
            self.detach_all_objects_from_gripper(whicharm)
            return

        obj = AttachedCollisionObject()
        obj.object.header.stamp = rospy.Time.now()
        obj.object.header.frame_id = "base_link"
        obj.link_name = whicharm+"_gripper_r_finger_tip_link"
        obj.object.id = object_collision_name
        obj.object.operation.operation = CollisionObjectOperation.DETACH_AND_ADD_AS_OBJECT
        self.attached_object_pub.publish(obj)


    ##convert a Pose message to a 4x4 scipy matrix
    def pose_to_mat(self, pose):
        quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        pos = scipy.matrix([pose.position.x, pose.position.y, pose.position.z]).T
        mat = scipy.matrix(tf.transformations.quaternion_matrix(quat))
        mat[0:3, 3] = pos
        return mat


    #convert a 4x4 scipy matrix to a Pose message
    def mat_to_pose(self, mat):
        pose = Pose()
        pose.position.x = mat[0,3]
        pose.position.y = mat[1,3]
        pose.position.z = mat[2,3]
        quat = tf.transformations.quaternion_from_matrix(mat)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose


    #adds the table to the map
    def process_collision_geometry_for_table(self, firsttable, additional_tables = []):

        table_object = CollisionObject()
        table_object.operation.operation = CollisionObjectOperation.ADD
        table_object.header.frame_id = firsttable.pose.header.frame_id
        table_object.header.stamp = rospy.Time.now()

        #create a box for each table
        for table in [firsttable,]+additional_tables:
            object = Shape()
            object.type = Shape.BOX;
            object.dimensions.append(math.fabs(table.x_max-table.x_min))
            object.dimensions.append(math.fabs(table.y_max-table.y_min))
            object.dimensions.append(0.01)
            table_object.shapes.append(object)
  
        #set the origin of the table object in the middle of the firsttable
        table_mat = self.pose_to_mat(firsttable.pose.pose)
        table_offset = scipy.matrix([(firsttable.x_min + firsttable.x_max)/2.0, (firsttable.y_min + firsttable.y_max)/2.0, 0.0]).T
        table_offset_mat = scipy.matrix(scipy.identity(4))
        table_offset_mat[0:3,3] = table_offset
        table_center = table_mat * table_offset_mat
        origin_pose = self.mat_to_pose(table_center)
        table_object.poses.append(origin_pose)

        table_object.id = "table"
        self.object_in_map_pub.publish(table_object)


    ##return the next unique collision object name
    def get_next_object_name(self):
        string = "graspable_object_"+str(self.object_in_map_current_id)
        self.object_in_map_current_id += 1
        return string


    ##adds a single box to the map
    def add_collision_box(self, box_pose, box_dims, frame_id, collision_name):
        
        rospy.loginfo("adding box to collision map")
        box = CollisionObject()
        box.operation.operation = CollisionObjectOperation.ADD
        box.header.frame_id = frame_id
        box.header.stamp = rospy.Time.now()
        shape = Shape()
        shape.type = Shape.BOX
        shape.dimensions = box_dims
        box.shapes.append(shape)
        box.poses.append(box_pose)
        box.id = collision_name
        self.object_in_map_pub.publish(box)


    ##adds a cluster to the map as a bunch of small boxes
    def process_collision_geometry_for_cluster(self, cluster):

        rospy.loginfo("adding cluster with %d points to collision map"%len(cluster.points))

        many_boxes = CollisionObject()
        
        many_boxes.operation.operation = CollisionObjectOperation.ADD
        many_boxes.header = cluster.header
        many_boxes.header.stamp = rospy.Time.now()
        num_to_use = int(len(cluster.points)/100.0)
        random_indices = range(len(cluster.points))
        scipy.random.shuffle(random_indices)
        random_indices = random_indices[0:num_to_use]
        for i in range(num_to_use):
            shape = Shape()
            shape.type = Shape.BOX
            shape.dimensions = [.005]*3
            pose = Pose()
            pose.position.x = cluster.points[random_indices[i]].x
            pose.position.y = cluster.points[random_indices[i]].y
            pose.position.z = cluster.points[random_indices[i]].z
            pose.orientation = Quaternion(*[0,0,0,1])
            many_boxes.shapes.append(shape)
            many_boxes.poses.append(pose)
        
        collision_name = self.get_next_object_name()
        many_boxes.id = collision_name
        self.object_in_map_pub.publish(many_boxes)
        return collision_name


    ##remove a single collision object from the map
    def remove_collision_object(self, collision_name):

        reset_object = CollisionObject()
        reset_object.operation.operation = CollisionObjectOperation.REMOVE
        reset_object.header.frame_id = "base_link"
        reset_object.header.stamp = rospy.Time.now()
        reset_object.id = collision_name
        self.object_in_map_pub.publish(reset_object) 

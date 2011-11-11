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

import smach
import smach_ros

import time

from object_manipulation_msgs.msg import GraspableObject
from object_manipulation_msgs.srv import GraspPlanning

from sensor_msgs.msg import PointCloud
from std_srvs.srv import Empty

class UnplugConnectorStateMachine(object):
    """
    """

    def __init__(self, ):
        """
        """
        rospy.init_node("unplug_connector")
        self.object_point_cloud_received = False

        self.point_cloud_subscriber = None
        self.grasp_planner_srv = None
        self.object_point_cloud = None

    def run(self):
        self.point_cloud_subscriber = rospy.Subscriber("~object_pcl_input", PointCloud, self.object_pcl_callback)

        rospy.wait_for_service('/sr_grasp_planner/plan_point_cluster_grasp')
        self.grasp_planner_srv = rospy.ServiceProxy( '/sr_grasp_planner/plan_point_cluster_grasp', GraspPlanning )

        self.start_service = rospy.Service('~start', Empty, self.plan_grasp)


        rospy.spin()

    def object_pcl_callback(self, msg):
        if not self.object_point_cloud_received:
            self.object_point_cloud_received = True

        self.object_point_cloud = msg

    def plan_grasp(self, req):
        while self.object_point_cloud == None:
            rospy.loginfo("Waiting to identify the object")
            time.sleep(0.1)

        graspable_object = GraspableObject()
        graspable_object.cluster = self.object_point_cloud

        rospy.loginfo("Detected object, trying to grasp it")

        try:
            resp1 = self.grasp_planner_srv( arm_name="", target=graspable_object, collision_object_name="",
                                            collision_support_surface_name="", grasps_to_evaluate = [])
        except rospy.ServiceException, e:
            rospy.logerr( "Service did not process request: %s"%str(e) )
            return []

        print resp1

        return []



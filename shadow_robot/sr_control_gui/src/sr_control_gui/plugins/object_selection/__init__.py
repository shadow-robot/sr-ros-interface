
#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_control_gui')
import rospy

from PyQt4 import QtCore, QtGui, Qt

from shadow_generic_plugin import ShadowGenericPlugin
from config import Config

from tabletop_object_detector.srv import TabletopDetection
from tabletop_object_detector.msg import TabletopDetectionResult
from household_objects_database_msgs.srv import GetModelDescription
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing
from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
import object_manipulator.draw_functions as draw_functions
from object_manipulator.convert_functions import *
from object_manipulation_msgs.msg import PickupGoal, PickupAction, PlaceGoal, PlaceAction
from geometry_msgs.msg import Vector3Stamped, PoseStamped, Pose
import actionlib
from actionlib_msgs.msg import *

from tf import transformations
import tf

class ObjectChooser(QtGui.QWidget):
    """
    Display the list of found objects
    """
    def __init__(self, parent, plugin_parent, title):
        QtGui.QWidget.__init__(self)
        self.plugin_parent = plugin_parent
        self.grasp = None
        self.title = QtGui.QLabel()
        self.title.setText(title)
        self.draw_functions = None
        self.pickup_result = None
        self.listener = tf.TransformListener()

    def draw(self):
        self.frame = QtGui.QFrame(self)

        self.tree = QtGui.QTreeWidget()
        self.connect(self.tree, QtCore.SIGNAL('itemDoubleClicked (QTreeWidgetItem *, int)'),
                     self.double_click)
        self.tree.setHeaderLabels(["Object Name", "Maker", "tags"])
        self.tree.resizeColumnToContents(0)
        self.tree.resizeColumnToContents(1)
        self.tree.resizeColumnToContents(2)

        self.layout = QtGui.QVBoxLayout()
        self.layout.addWidget(self.title)
        self.layout.addWidget(self.tree)

        self.draw_functions = draw_functions.DrawFunctions('grasp_markers')
        ###
        # SIGNALS
        ##
        self.plugin_parent.parent.parent.reload_object_signal_widget.reloadObjectSig['int'].connect(self.refresh_list)

        self.frame.setLayout(self.layout)
        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.frame)
        self.frame.show()
        self.setLayout(layout)
        self.show()

    def double_click(self, item, value):
        object_name = str(item.text(0))
        self.object = self.plugin_parent.found_objects[object_name]

        graspable_object = self.object.graspable_object

        # draw a bounding box around the selected object
        (box_pose, box_dims) = self.call_find_cluster_bounding_box(graspable_object.cluster)
        if box_pose == None:
            return

        box_mat = pose_to_mat(box_pose.pose)
        box_ranges = [[-box_dims.x / 2, -box_dims.y / 2, -box_dims.z / 2],
                      [box_dims.x / 2, box_dims.y / 2, box_dims.z / 2]]

        self.draw_functions.draw_rviz_box(box_mat, box_ranges, '/base_link',
                                          ns='bounding box',
                                          color=[0, 0, 1], opaque=0.25, duration=60)

        # call the pickup service
        res = self.pickup(graspable_object, self.object.graspable_object_name, object_name)

        if res == 0: #correctly picked up
        #TODO: set up place_location
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

            list_of_poses = self.compute_list_of_poses(initial_pose, graspable_object)

            self.place_object(graspable_object, self.object.graspable_object_name, object_name, list_of_poses)

    def pickup(self, graspable_object, graspable_object_name, object_name):
        info_tmp = "Picking up "+ object_name
        rospy.loginfo(info_tmp)
        pickup_goal = PickupGoal()
        pickup_goal.target = graspable_object
        pickup_goal.collision_object_name = graspable_object_name
        pickup_goal.collision_support_surface_name = self.plugin_parent.collision_support_surface_name

        pickup_goal.arm_name = "right_arm"
        pickup_goal.desired_approach_distance = 0.05
        pickup_goal.min_approach_distance = 0.02

        direction = Vector3Stamped()
        direction.header.stamp = rospy.get_rostime()
        direction.header.frame_id = "/base_link";
        direction.vector.x = 0;
        direction.vector.y = 0;
        direction.vector.z = 1;
        pickup_goal.lift.direction = direction;
        #request a vertical lift of 15cm after grasping the object
        pickup_goal.lift.desired_distance = 0.25;
        pickup_goal.lift.min_distance = 0.2;
        #do not use tactile-based grasping or tactile-based lift
        pickup_goal.use_reactive_lift = True;
        pickup_goal.use_reactive_execution = True;

        pickup_client = actionlib.SimpleActionClient('/object_manipulator/object_manipulator_pickup', PickupAction)
        pickup_client.wait_for_server()
        rospy.loginfo("Pickup server ready")

        pickup_client.send_goal(pickup_goal)
        #timeout after 1sec
        #TODO: change this when using the robot
        pickup_client.wait_for_result(timeout=rospy.Duration.from_sec(90.0))
        rospy.loginfo("Got Pickup results")

        self.pickup_result = pickup_client.get_result()

        if pickup_client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr("The pickup action has failed: " + str(self.pickup_result.manipulation_result.value) )
            return -1

        return 0

    def place_object(self, graspable_object, graspable_object_name, object_name, list_of_poses ):
        """
        place the given object in the given pose
        """
        if self.pickup_result == None:
            rospy.logwarn("No objects where picked up. Aborting place object action.")
            return

        info_tmp = "Placing "+object_name
        rospy.loginfo(info_tmp)

        place_goal = PlaceGoal()

        #place at the prepared location


        place_goal.place_locations = list_of_poses

        place_goal.collision_object_name = graspable_object_name
        place_goal.collision_support_surface_name = self.plugin_parent.collision_support_surface_name
        print "collision support surface name: ",self.plugin_parent.collision_support_surface_name

        #information about which grasp was executed on the object,
        #returned by the pickup action
        place_goal.grasp = self.pickup_result.grasp
        #use the right rm to place
        place_goal.arm_name = "right_arm"
        #padding used when determining if the requested place location
        #would bring the object in collision with the environment
        place_goal.place_padding = 0.01
        #how much the gripper should retreat after placing the object
        place_goal.desired_retreat_distance = 0.1
        place_goal.min_retreat_distance = 0.05
        #we will be putting down the object along the "vertical" direction
        #which is along the z axis in the base_link frame
        direction = Vector3Stamped()
        direction.header.stamp = rospy.get_rostime()
        direction.header.frame_id = "/base_link"
        direction.vector.x = 0
        direction.vector.y = 0
        direction.vector.z = -1
        place_goal.approach.direction = direction
        #request a vertical put down motion of 10cm before placing the object
        place_goal.approach.desired_distance = 0.1
        place_goal.approach.min_distance = 0.05
        #we are not using tactile based placing
        place_goal.use_reactive_place = False

        place_client = actionlib.SimpleActionClient('/object_manipulator/object_manipulator_place', PlaceAction)
        place_client.wait_for_server()
        rospy.loginfo("Place server ready")

        place_client.send_goal(place_goal)
        #timeout after 1sec
        #TODO: change this when using the robot
        place_client.wait_for_result(timeout=rospy.Duration.from_sec(90.0))
        rospy.loginfo("Got Place results")

        place_result = place_client.get_result()

        if place_client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr("The place action has failed: " + str(place_result.manipulation_result.value) )
        print place_result

    def compute_list_of_poses(self, initial_pose, graspable_object, rect_w=0.20, rect_h=0.20, resolution=0.02):
        '''
        Computes a list of possible poses in a rectangle of 2*rect_w by 2*rect_h, with the given resolution.
        In our case, rect_w is along the x axis, and rect_h along the y_axis.
        '''
        list_of_poses = []

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

        self.draw_place_area(list_of_poses, graspable_object)

        return list_of_poses

    def draw_place_area(self, list_of_poses, graspable_object):
        '''
        Displays all the possible placing locations that are going to be tried.
        '''
        #try:
        (trans_palm,rot_palm) = self.listener.lookupTransform('/base_link', '/palm', rospy.Time(0))
        #except:
        #    return

        for index,pose_stamped in enumerate(list_of_poses):
            pose_tmp = Pose()
            pose_tmp.position.x = pose_stamped.pose.position.x + trans_palm[0]
            pose_tmp.position.y = pose_stamped.pose.position.y + trans_palm[1]
            pose_tmp.position.z = 0.01

            pose_tmp.orientation.x = pose_stamped.pose.orientation.x
            pose_tmp.orientation.y = pose_stamped.pose.orientation.y
            pose_tmp.orientation.z = pose_stamped.pose.orientation.z
            pose_tmp.orientation.w = pose_stamped.pose.orientation.w

            mat = pose_to_mat(pose_tmp)
            self.draw_functions.draw_rviz_box(mat, [.01,.01,.01], frame='/base_link', ns='place_'+str(index),
                                              id=1000+index, duration = 90, color=[0.5,0.5,0.0], opaque=1.0 )


    def call_find_cluster_bounding_box(self, cluster):
        req = FindClusterBoundingBoxRequest()
        req.cluster = cluster
        service_name = "find_cluster_bounding_box"
        rospy.loginfo("waiting for find_cluster_bounding_box service")
        rospy.wait_for_service(service_name)
        rospy.loginfo("service found")
        serv = rospy.ServiceProxy(service_name, FindClusterBoundingBox)
        try:
            res = serv(req)
        except rospy.ServiceException, e:
            rospy.logerr("error when calling find_cluster_bounding_box: %s" % e)
            return 0
        if not res.error_code:
            return (res.pose, res.box_dims)
        else:
            return (None, None)


    def refresh_list(self, value=0):
        self.tree.clear()
        first_item = None
        object_names = self.plugin_parent.found_objects.keys()
        object_names.sort()
        for object_name in object_names:
            item = QtGui.QTreeWidgetItem(self.tree)
            if first_item == None:
                first_item = item

            item.setText(0, object_name)
            obj = self.plugin_parent.found_objects[object_name].model_description
            if "unknown_" not in object_name:
                item.setText(1, obj.maker)

                tags = ""
                for tag in obj.tags:
                    tags += str(tag) + " ; "
                item.setText(2, tags)

            self.tree.resizeColumnToContents(0)
            self.tree.resizeColumnToContents(1)
            self.tree.resizeColumnToContents(2)

            #print "add"
            #self.tree.addTopLevelItem(item)
        return first_item

class TableObject(object):
    """
    Contains all the relevant info for an object.
    A map of TableObject is stored in the ObjectSelection plugin.
    """
    def __init__(self):
        self.graspable_object = None
        self.graspable_object_name = None
        self.model_description = None

class Model(object):
    def __init__(self):
        self.name = None

class ObjectSelection(ShadowGenericPlugin):
    """
    Contact the tabletop object detector to get a list of the objects on top
    of the table. Then possibility to select a detected object for picking it up.
    """
    name = "Object Selection"

    def __init__(self):
        ShadowGenericPlugin.__init__(self)

        self.service_object_detector = None
        self.service_db_get_model_description = None
        self.service_tabletop_collision_map = None
        self.found_objects = {}
        self.raw_objects = None
        self.collision_support_surface_name = None
        self.unknown_object_counter = 1 #starts at 1 as it's only used for display

        self.frame = QtGui.QFrame()
        self.layout = QtGui.QVBoxLayout()
        subframe = QtGui.QFrame()
        sublayout = QtGui.QHBoxLayout()
        self.btn_refresh = QtGui.QPushButton()
        self.btn_refresh.setText("Detect Objects")
        self.btn_refresh.setFixedWidth(130)
        self.frame.connect(self.btn_refresh, QtCore.SIGNAL('clicked()'), self.detect_objects)

        self.btn_collision_map = QtGui.QPushButton()
        self.btn_collision_map.setText("Refresh Collision Map")
        self.frame.connect(self.btn_collision_map, QtCore.SIGNAL('clicked()'), self.process_collision_map)
        self.btn_collision_map.setDisabled(True)
        sublayout.addWidget(self.btn_refresh)
        sublayout.addWidget(self.btn_collision_map)
        subframe.setLayout(sublayout)
        self.layout.addWidget(subframe)

        self.object_chooser = ObjectChooser(self.window, self, "Objects Detected")
        self.layout.addWidget(self.object_chooser)

        self.frame.setLayout(self.layout)
        self.window.setWidget(self.frame)

        self.is_activated = False

    def activate(self):
        ShadowGenericPlugin.activate(self)
        if self.service_object_detector == None:
            rospy.wait_for_service('object_detection')
            self.service_object_detector = rospy.ServiceProxy('object_detection', TabletopDetection)

        if self.service_db_get_model_description == None:
            rospy.wait_for_service('objects_database_node/get_model_description')
            self.service_db_get_model_description = rospy.ServiceProxy('objects_database_node/get_model_description', GetModelDescription)

        if self.service_tabletop_collision_map == None:
            rospy.wait_for_service('/tabletop_collision_map_processing/tabletop_collision_map_processing')
            self.service_tabletop_collision_map = rospy.ServiceProxy('/tabletop_collision_map_processing/tabletop_collision_map_processing', TabletopCollisionMapProcessing)

        self.object_chooser.draw()

    def get_object_name(self, model_id):
        """
        return the object name given its index (read from database, or
        create unique name if unknown object).
        """
        model = Model()
        #todo make sure name is unique
        if model_id == -1:
            model.name = "unknown_" + str(self.unknown_object_counter)
            self.unknown_object_counter += 1
        else:
            try:
                model = self.service_db_get_model_description(model_id)
            except rospy.ServiceException, e:
                print "Service did not process request: %s" % str(e)
                model.name = "unkown_recognition_failed"
        return model

    def detect_objects(self):
        self.found_objects.clear()
        try:
            self.raw_objects = self.service_object_detector(True, True, 1)
        except rospy.ServiceException, e:
            print "Service did not process request: %s" % str(e)

        #take a new collision map + add the detected objects to the collision map and get graspable objects from them
        tabletop_collision_map_res = self.process_collision_map()

        print tabletop_collision_map_res.collision_object_names

        if tabletop_collision_map_res != 0:
            for grasp_obj, grasp_obj_name in zip(tabletop_collision_map_res.graspable_objects, tabletop_collision_map_res.collision_object_names):
                obj_tmp = TableObject()
                obj_tmp.graspable_object = grasp_obj
                obj_tmp.graspable_object_name = grasp_obj_name

                if len(grasp_obj.potential_models) > 0:
                    model_index = grasp_obj.potential_models[0].model_id
                else:
                    model_index = -1
                obj_tmp.model_description = self.get_object_name(model_index)


                self.found_objects[obj_tmp.model_description.name] = obj_tmp

        if self.raw_objects != None:
            self.btn_collision_map.setEnabled(True)
        self.parent.parent.reload_object_signal_widget.reloadObjectSig['int'].emit(1)

    def process_collision_map(self):
        res = 0
        try:
            #reset_static_map, reset_collision_models, reset_attached_models, take_static_collision_map
            res = self.service_tabletop_collision_map.call(self.raw_objects.detection, True, True, True, True, "/base_link")
        except rospy.ServiceException, e:
            rospy.logerr("Service did not process request: %s" % str(e))

        if res != 0:
            self.collision_support_surface_name = res.collision_support_surface_name
        return res

    def on_close(self):
        GenericPlugin.on_close(self)

    def depends(self):
        return Config.sr_object_selection_config.dependencies


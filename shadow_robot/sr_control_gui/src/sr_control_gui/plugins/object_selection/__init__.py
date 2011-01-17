#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_control_gui')
import rospy

from PyQt4 import QtCore, QtGui, Qt

from generic_plugin import GenericPlugin
from config import Config

from tabletop_object_detector.srv import TabletopDetection
from tabletop_object_detector.msg import TabletopDetectionResult
from household_objects_database_msgs.srv import GetModelDescription
from tabletop_collision_map_processing.srv import TabletopCollisionMapProcessing
from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
import object_manipulator.draw_functions as draw_functions
from object_manipulator.convert_functions import *

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
        self.object = self.plugin_parent.found_objects[str(item.text(0))]
        
        graspable_object = self.object.graspable_object
        (box_pose, box_dims) = self.call_find_cluster_bounding_box(graspable_object.cluster)
        if box_pose == None:
            return
        
        box_mat = pose_to_mat(box_pose.pose)
        box_ranges = [[-box_dims.x / 2, -box_dims.y / 2, -box_dims.z / 2],
                      [box_dims.x / 2, box_dims.y / 2, box_dims.z / 2]]

        self.draw_functions.draw_rviz_box(box_mat, box_ranges, 'base_link',
                                          ns='bounding box',
                                          color=[0, 0, 1], opaque=0.25, duration=60)
                
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
    
class ObjectSelection(GenericPlugin):  
    """
    Contact the tabletop object detector to get a list of the objects on top 
    of the table. Then possibility to select a detected object for picking it up.
    """
    name = "Object Selection"
        
    def __init__(self):
        GenericPlugin.__init__(self)

        self.service_object_detector = None
        self.service_db_get_model_description = None
        self.service_tabletop_collision_map = None
        self.found_objects = {}
        self.number_of_unrecognized_objects = 0

        self.frame = QtGui.QFrame()
        self.layout = QtGui.QVBoxLayout()
        self.btn_refresh = QtGui.QPushButton()
        self.btn_refresh.setText("Detect Objects")
        self.btn_refresh.setFixedWidth(130)
        self.frame.connect(self.btn_refresh, QtCore.SIGNAL('clicked()'), self.detect_objects)
        self.layout.addWidget(self.btn_refresh)
        
        self.object_chooser = ObjectChooser(self.window, self, "Objects Detected")
        self.layout.addWidget(self.object_chooser)
        
        self.frame.setLayout(self.layout)
        self.window.setWidget(self.frame)

        self.is_activated = False
    
    def activate(self):
        if self.is_activated:
            return
        self.is_activated = True

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


        GenericPlugin.activate(self)

    def get_object_name(self, model_id):
        """
        return the object name given its index (read from database, or 
        create unique name if unknown object).
        """
        model = Model()
        #todo make sure name is unique
        if model_id == -1:
            model.name = "unknown_"
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
            objects = self.service_object_detector(True, True)
        except rospy.ServiceException, e:
            print "Service did not process request: %s" % str(e)
        
        #take a new collision map + add the detected objects to the collision map and get graspable objects from them 
        tabletop_collision_map_res = self.process_collision_map(objects.detection)
        
        if tabletop_collision_map_res != 0:
            for grasp_obj, grasp_obj_name in zip(tabletop_collision_map_res.graspable_objects, tabletop_collision_map_res.collision_object_names):
                obj_tmp = TableObject()
                obj_tmp.graspable_object = grasp_obj
                obj_tmp.graspable_object_name = grasp_obj_name
                
                model_index = grasp_obj.model_pose.model_id
                obj_tmp.model_description = self.get_object_name(model_index)
                
                
                self.found_objects[obj_tmp.model_description.name] = obj_tmp
            
        self.parent.parent.reload_object_signal_widget.reloadObjectSig['int'].emit(1)
        
    def process_collision_map(self, detection):
        res = 0
        try:
            #reset_static_map, reset_collision_models, reset_attached_models, take_static_collision_map
            res = self.service_tabletop_collision_map.call(detection, True, True, True, True, "base_link")
        except rospy.ServiceException, e:
            print "Service did not process request: %s" % str(e)
        
        return res
    
    def on_close(self):
        GenericPlugin.on_close(self)

    def depends(self):
        return Config.sr_object_selection_config.dependencies
                

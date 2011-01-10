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
        print str(item.text(0)), " double clicked"
                
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
            obj = self.plugin_parent.found_objects[object_name]
            if "unrecognized_" not in object_name:
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
        self.draw_functions = None
        self.graspable_objects = None

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

        self.draw_functions = draw_functions.DrawFunctions('grasp_markers')

        GenericPlugin.activate(self)

    def detect_objects(self):
        self.found_objects.clear()
        try:
            objects = self.service_object_detector(True, True)
        except rospy.ServiceException, e:
            print "Service did not process request: %s" % str(e)
        
        self.number_of_unrecognized_objects = 0
        
        for index, cmi in zip(range(0, len(objects.detection.cluster_model_indices)), objects.detection.cluster_model_indices):
            # object not recognized
            if cmi == -1:
                self.number_of_unrecognized_objects += 1
                tmp_name = "unrecognized_" + str(self.number_of_unrecognized_objects)
                #TODO: change this
                self.found_objects[tmp_name] = objects.detection.clusters[0]
                
        # for the recognized objects
        for model in objects.detection.models:
            model_id = model.model_id
            
            try:
                model_desc = self.service_db_get_model_description(model_id)
            except rospy.ServiceException, e:
                print "Service did not process request: %s" % str(e)
            
            self.found_objects[model_desc.name] = model_desc
        
        self.parent.parent.reload_object_signal_widget.reloadObjectSig['int'].emit(1)
        
        tabletop_collision_map_res = self.process_collision_map(objects.detection)
        
        if tabletop_collision_map_res != 0:
            self.graspable_objects = tabletop_collision_map_res.graspable_objects
    
            rospy.loginfo("Graspable object found")
                
            if self.graspable_objects != 0:
                if len(self.graspable_objects) > 0: 
                    for graspable_object in self.graspable_objects:
                        (box_pose, box_dims) = self.call_find_cluster_bounding_box(graspable_object.cluster)
                        if box_pose == None:
                            return
        
                        box_mat = pose_to_mat(box_pose.pose)
                        box_ranges = [[-box_dims.x/2, -box_dims.y/2, -box_dims.z/2],
                                      [box_dims.x/2, box_dims.y/2, box_dims.z/2]]
        
                        self.draw_functions.draw_rviz_box(box_mat, box_ranges, 'base_link', 
                                                          ns = 'bounding box', 
                                                          color = [0,0,1], opaque = 0.25, duration = 60)


    def process_collision_map(self, detection):
        res = 0
        try:
            #reset_static_map, reset_collision_models, reset_attached_models, take_static_collision_map
            res = self.service_tabletop_collision_map.call(detection, False, True, True, False, "base_link")
        except rospy.ServiceException, e:
            print "Service did not process request: %s" % str(e)
        
        return res

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
            rospy.logerr("error when calling find_cluster_bounding_box: %s"%e)
            return 0
        if not res.error_code:
            return (res.pose, res.box_dims)
        else:
            return (None, None)
        

    
    def on_close(self):
        GenericPlugin.on_close(self)

    def depends(self):
        return Config.sr_object_selection_config.dependencies
                

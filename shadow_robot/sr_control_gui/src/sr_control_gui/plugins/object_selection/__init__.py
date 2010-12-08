#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_control_gui')
import rospy

from PyQt4 import QtCore, QtGui, Qt

from generic_plugin import GenericPlugin
from config import Config

from tabletop_object_detector.srv import TabletopDetection
from tabletop_object_detector.msg import TabletopDetectionResult
from household_objects_database_msgs.srv import GetModelDescription


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

        self.list = QtGui.QListWidget()
        first_item = self.refresh_list()  
        self.connect(self.list, QtCore.SIGNAL('itemClicked(QListWidgetItem*)'), self.object_choosed)
            
        self.connect(self.list, QtCore.SIGNAL('itemDoubleClicked(QListWidgetItem*)'), self.double_click)
        self.list.setViewMode(QtGui.QListView.ListMode)
        self.list.setResizeMode(QtGui.QListView.Adjust)
        self.list.setItemSelected(first_item, True)
        #self.object_choosed(first_item, first_time=True)
        
        self.layout = QtGui.QVBoxLayout()
        self.layout.addWidget(self.title)
        self.layout.addWidget(self.list)
        
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
        
    def double_click(self, item):
        self.object = self.plugin_parent.found_objects[str(item.text())]
        print str(item.text())
    
    def object_choosed(self, item, first_time=False):
        self.object = self.plugin_parent.found_objects[str(item.text())]
            
    def refresh_list(self, value = 0):
        self.list.clear()   
        first_item = None
        tmp = self.plugin_parent.found_objects.keys()
        tmp.sort()
        for object_name in tmp:
            item = QtGui.QListWidgetItem(object_name)
            if first_item == None:
                first_item = item
            self.list.addItem(item)
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
    
    def detect_objects(self):
        self.found_objects.clear()
        try:
            objects = self.service_object_detector(True, True)
        except rospy.ServiceException, e:
            print "Service did not process request: %s"%str(e)
        
        self.number_of_unrecognized_objects = 0
        for index, cmi in zip(range(0, len(objects.detection.cluster_model_indices)), objects.detection.cluster_model_indices):
            # object not recognized
            if cmi == -1:
                self.number_of_unrecognized_objects += 1
                tmp_name = "unrecognized_"+str(self.number_of_unrecognized_objects)
                self.found_objects[tmp_name] = objects.detection[index]
        
        # for the recognized objects
        for model in objects.detection.models:
            model_id = model.model_id
            
            try:
                model_desc = self.service_db_get_model_description(model_id)
            except rospy.ServiceException, e:
                print "Service did not process request: %s"%str(e)
            
            self.found_objects[model_desc.name] = model_desc
        
        
        self.parent.parent.reload_object_signal_widget.reloadObjectSig['int'].emit(1)
        #print self.found_objects
            
            
    
    def activate(self):
        
        if self.is_activated:
            return
        self.is_activated = True

        if self.service_object_detector == None:
            rospy.wait_for_service('object_detection', 1.0)
            self.service_object_detector = rospy.ServiceProxy('object_detection', TabletopDetection)

        if self.service_db_get_model_description == None:
            rospy.wait_for_service('objects_database_node/get_model_description', 1.0)
            self.service_db_get_model_description = rospy.ServiceProxy('objects_database_node/get_model_description', GetModelDescription)

        self.object_chooser.draw()

        GenericPlugin.activate(self)

    def on_close(self):
        GenericPlugin.on_close(self)

    def depends(self):
        return Config.sr_object_selection_config.dependencies
                

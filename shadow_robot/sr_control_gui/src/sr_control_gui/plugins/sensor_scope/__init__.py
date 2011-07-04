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


import roslib; roslib.load_manifest('sr_control_gui')
import rospy

from open_gl_generic_plugin import OpenGLGenericPlugin
from utils.rostopic import RosTopicChecker
from config import Config
import math

from PyQt4 import QtCore, QtGui, Qt
from OpenGL.GL import *
from OpenGL.GLU import *
from PyQt4 import QtGui
from PyQt4.QtOpenGL import *

from collections import deque
from std_msgs.msg import Int16



class DataSet(object):
    default_color = QtCore.Qt.black

    def __init__(self, parent, index = 0):
        self.parent = parent

        self.index = index

        self.points = []
        self.lines = []
        self.pen = QtGui.QPen(self.default_color, 1, 
                              QtCore.Qt.SolidLine, QtCore.Qt.RoundCap, 
                              QtCore.Qt.RoundJoin)
        self.init_dataset()

    def init_dataset(self):
        for index_points in range(0, self.parent.number_of_points):
            tmp_line = QtGui.QGraphicsLineItem(index_points,self.index*100,index_points + 1,self.index*100)
            tmp_line.setPen(self.pen)
            self.parent.scene.addItem(tmp_line)
            self.lines.append(tmp_line)

            tmp = [0]* self.parent.number_of_points
            self.points = deque(tmp)

    def change_color(self, r,g,b):
        self.pen = QtGui.QPen(Qt.QColor.fromRgb(r,g,b), 1, 
                              QtCore.Qt.SolidLine, QtCore.Qt.RoundCap, 
                              QtCore.Qt.RoundJoin)
        for line in self.lines:
            line.setPen(self.pen)

class SubscribeTopicFrame(QtGui.QFrame):
    """
    """
    
    def __init__(self, parent, subscriber_index):
        """
        """
        self.parent = parent
        self.subscriber_index = subscriber_index
        QtGui.QFrame.__init__(self)
        self.layout = QtGui.QHBoxLayout()
        
        self.topic_box = QtGui.QComboBox(self)

        self.topic_box.addItem("None")
        for pub in parent.all_pubs:
            self.topic_box.addItem(pub[0])
        self.connect(self.topic_box, QtCore.SIGNAL('activated(int)'), self.onChanged)
        self.layout.addWidget(self.topic_box)
        self.setLayout(self.layout)

    def onChanged(self, index):
        text = self.topic_box.currentText()
        #unsubscribe the current topic
        self.parent.remove_subscriber(self.subscriber_index)

        if text != "None":
            self.parent.add_subscriber(str(text), Int16, self.subscriber_index)
        

class SensorScope(OpenGLGenericPlugin):
    """
    Plots some chosen debug values.
    """
    name = "Sensor Scope"

    def __init__(self):
        OpenGLGenericPlugin.__init__(self, self.paint_method)
        self.subscribers = []
        self.datasets = []
        self.points_size = self.open_gl_widget.number_of_points
        
        self.all_pubs = None
        self.topic_checker = RosTopicChecker()
        self.refresh_topics()

        self.control_layout = QtGui.QVBoxLayout()

        self.play_btn = QtGui.QPushButton()
        self.play_btn.setFixedWidth(30)
        self.control_frame.connect(self.play_btn, QtCore.SIGNAL('clicked()'), self.button_play_clicked)
        self.control_layout.addWidget(self.play_btn)

        self.subscribe_topic_frames = []
        for i in range (0,4):
            tmp_stf = SubscribeTopicFrame(self, i)
            self.control_layout.addWidget( tmp_stf )
            self.subscribe_topic_frames.append( tmp_stf )

        self.control_frame.setLayout(self.control_layout)

        self.paused = False


    def activate(self):
        OpenGLGenericPlugin.activate(self)
        self.play_btn.setIcon(QtGui.QIcon(self.parent.parent.rootPath + '/images/icons/pause.png'))

        for i in range (0,4):
            self.add_subscriber("/wait", Int16, i)
            self.subscribers[i].unregister()

    def remove_subscriber(self, index):
        self.subscribers[index].unregister()

    def add_subscriber( self, topic, msg_type, index ):
        if index >= len(self.subscribers):
            self.subscribers.append( rospy.Subscriber(topic, msg_type, self.msg_callback, len(self.subscribers)) )
        else:
            self.subscribers[index] = rospy.Subscriber(topic, msg_type, self.msg_callback, index) 
        tmp_dataset = DataSet(self.open_gl_widget, index = -1)
        tmp_dataset.change_color(10*(len(self.datasets)%25),100*(len(self.datasets)%2),70*(len(self.datasets)%3))
        self.datasets.append(tmp_dataset)

        self.open_gl_widget.center_at_the_end()


    def msg_callback(self, msg, index):
        """
        Received a message on the topic. Adding the new point to
        the queue.
        """
        #received message from subscriber at index: update the last
        # point and pop the first point
        #print index, " ", msg.data
        for data_set_id,data_set in enumerate(self.datasets):
            if data_set_id == index:
                data_set.points.append(msg.data)
            else:
                data_set.points.append(data_set.points[-1])
            data_set.points.popleft()

    def paint_method(self):
        '''
        Drawing routine: this function is called periodically.
        '''
        if self.paused:
            return
        for data_set_id,data_set in enumerate(self.datasets):
            for index in range(0, self.points_size - 1):
                # invert the data because the frame is pointing downwards.
                data_set.lines[index].setLine(index, - data_set.points[index],
                                              index + 1, - data_set.points[index + 1])

    def button_play_clicked(self):
        #lock mutex here
        if not self.paused:
            self.paused = True
            self.play_btn.setIcon(QtGui.QIcon(self.parent.parent.rootPath + '/images/icons/play.png'))
        else:
            self.paused = False
            self.play_btn.setIcon(QtGui.QIcon(self.parent.parent.rootPath + '/images/icons/pause.png'))

    def refresh_topics(self):
        self.all_pubs = self.topic_checker.get_topics()

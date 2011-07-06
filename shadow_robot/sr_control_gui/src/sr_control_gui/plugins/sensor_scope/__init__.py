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
from OpenGL.GLUT import *
from PyQt4 import QtGui
from PyQt4.QtOpenGL import *

from collections import deque
from std_msgs.msg import Int16



class DataSet(object):
    default_color = QtCore.Qt.black

    def __init__(self, parent, index = 0):
        self.parent = parent

        self.enabled = False

        self.index = index

        self.points = []
        self.init_dataset()
        self.scaled_color = []
        self.raw_color = []

        self.change_color(Qt.QColor(255,0,0))

    def init_dataset(self):
        for index_points in range(0, self.parent.number_of_points):
            tmp = [0]* self.parent.number_of_points
            self.points = deque(tmp)

    def change_color(self, col):
        r = col.redF()
        g = col.greenF()
        b = col.blueF()

        self.scaled_color = [r, g, b]

        #create a darker version of the raw color
        r *= 0.5
        g *= 0.5
        b *= 0.5
        self.raw_color = [r,g,b]

    def get_raw_color(self):
        return self.raw_color

    def get_scaled_color(self):
        return self.scaled_color


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

        #add a combo box to select the topic
        self.topic_box = QtGui.QComboBox(self)
        self.topic_box.addItem("None")
        for pub in parent.all_pubs:
            self.topic_box.addItem(pub[0])
        self.connect(self.topic_box, QtCore.SIGNAL('activated(int)'), self.onChanged)
        self.layout.addWidget(self.topic_box)

        #add a button to change the color
        self.change_color_btn = QtGui.QPushButton()
        self.change_color_btn.setIcon(QtGui.QIcon(self.parent.parent.parent.rootPath + '/images/icons/color_wheel.png'))
        self.change_color_btn.setFixedWidth(30)
        self.connect(self.change_color_btn, QtCore.SIGNAL('clicked()'), self.change_color_clicked)
        self.layout.addWidget(self.change_color_btn)

        #add a button to add a subscribe topic frame
        self.add_subscribe_topic_btn = QtGui.QPushButton()
        self.add_subscribe_topic_btn.setText('+')
        self.add_subscribe_topic_btn.setFixedWidth(30)
        self.connect(self.add_subscribe_topic_btn, QtCore.SIGNAL('clicked()'),self.add_subscribe_topic_clicked)
        self.layout.addWidget(self.add_subscribe_topic_btn)

        #add a button to remove the current subscribe topic frame
        if len(self.parent.subscribers) > 0:
            self.remove_topic_btn = QtGui.QPushButton()
            self.remove_topic_btn.setText('-')
            self.remove_topic_btn.setFixedWidth(30)
            self.connect(self.remove_topic_btn, QtCore.SIGNAL('clicked()'),self.remove_topic_clicked)
            self.layout.addWidget(self.remove_topic_btn)

        self.setLayout(self.layout)

    def onChanged(self, index):
        text = self.topic_box.currentText()
        #unsubscribe the current topic
        self.parent.remove_subscriber(self.subscriber_index)

        if text != "None":
            self.parent.datasets[self.subscriber_index].enabled = True
            self.parent.add_subscriber(str(text), Int16, self.subscriber_index)
        else:
            self.parent.datasets[self.subscriber_index].enabled = False

    def change_color_clicked(self):
        col = QtGui.QColorDialog.getColor()
        if col.isValid():
            self.parent.datasets[self.subscriber_index].change_color(col)

    def add_subscribe_topic_clicked(self):
        self.parent.add_topic_subscriber()
        Qt.QTimer.singleShot(0, self.adjustSize)

    def remove_topic_clicked(self):
        self.parent.subscribers[self.subscriber_index].unregister()
        self.parent.subscribers.remove(self.parent.subscribers[self.subscriber_index])
        self.parent.subscribe_topic_frames.remove(self)
        Qt.QTimer.singleShot(0, self.adjustSize)
        del self

class SensorScope(OpenGLGenericPlugin):
    """
    Plots some chosen debug values.
    """
    name = "Sensor Scope"

    def __init__(self):
        OpenGLGenericPlugin.__init__(self, self.paint_method)
        self.subscribers = []
        self.datasets = []
        self.data_points_size = self.open_gl_widget.number_of_points

        self.all_pubs = None
        self.topic_checker = RosTopicChecker()
        self.refresh_topics()

        self.control_layout = QtGui.QVBoxLayout()

        self.play_btn = QtGui.QPushButton()
        self.play_btn.setFixedWidth(30)
        self.control_frame.connect(self.play_btn, QtCore.SIGNAL('clicked()'), self.button_play_clicked)
        self.control_layout.addWidget(self.play_btn)

        self.subscribe_topic_frames = []

        self.paused = False


    def activate(self):
        OpenGLGenericPlugin.activate(self)
        self.play_btn.setIcon(QtGui.QIcon(self.parent.parent.rootPath + '/images/icons/pause.png'))

        self.add_topic_subscriber()

        self.control_frame.setLayout(self.control_layout)

    def add_topic_subscriber(self):
        index = len(self.subscribe_topic_frames)
        tmp_stf = SubscribeTopicFrame(self, index)
        self.control_layout.addWidget( tmp_stf )
        self.subscribe_topic_frames.append( tmp_stf )

        self.add_subscriber("/wait", Int16, index)
        self.subscribers[index].unregister()

    def remove_subscriber(self, index):
        self.subscribers[index].unregister()

    def add_subscriber( self, topic, msg_type, index ):
        if index >= len(self.subscribers):
            self.subscribers.append( rospy.Subscriber(topic, msg_type, self.msg_callback, len(self.subscribers)) )
        else:
            self.subscribers[index] = rospy.Subscriber(topic, msg_type, self.msg_callback, index)
        tmp_dataset = DataSet(self.open_gl_widget, index = -1)
        self.datasets.append(tmp_dataset)


    def msg_callback(self, msg, index):
        """
        Received a message on the topic. Adding the new point to
        the queue.
        """
        #received message from subscriber at index: update the last
        # point and pop the first point
        #print index, " ", msg.data
        for data_set_id,data_set in enumerate(self.datasets):
            if data_set.enabled:
                if data_set_id == index:
                    data_set.points.append(msg.data)
                else:
                    data_set.points.append(data_set.points[-1])
            else:
                # if the data set is not enabled (i.e. subscribing to None),
                # we only draw a line on 0
                data_set.points.append(0)
            data_set.points.popleft()

    def paint_method(self):
        '''
        Drawing routine: this function is called periodically.
        '''
        if self.paused:
            return

        display_points = []
        colors = []

        for data_set_id,data_set in enumerate(self.datasets):
            #we want to keep the last point of the raw data in the middle of
            # the screen
            offset = self.open_gl_widget.height/2 - data_set.points[-1]

            for display_index in range(0, self.open_gl_widget.number_of_points_to_display):
                # add the raw data
                colors.append(data_set.get_raw_color())
                data_index = self.display_to_data_index(display_index)
                display_points.append([display_index, data_set.points[data_index] + offset])

                #also add the scaled data
                colors.append(data_set.get_scaled_color())
                display_points.append([display_index, self.scale_data(data_set.points[data_index])] )

        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_COLOR_ARRAY)
        glColorPointerf(colors)
        glVertexPointerf(display_points)
        glClear(GL_COLOR_BUFFER_BIT)
        #glDrawArrays(GL_LINE_STRIP, 0, len(display_points))
        glPointSize(1.5)
        #glEnable(GL_POINT_SMOOTH)
        #glEnable(GL_BLEND)
        glDrawArrays(GL_POINTS, 0, len(display_points))

        glDisableClientState(GL_VERTEX_ARRAY)
        glDisableClientState(GL_COLOR_ARRAY)
        glFlush()

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

    def display_to_data_index(self, display_index):
        data_index = self.data_points_size - self.open_gl_widget.number_of_points_to_display + display_index
        return data_index

    def scale_data(self, data, data_max = 65536):
        scaled_data = (data * self.open_gl_widget.height) / data_max
        return scaled_data

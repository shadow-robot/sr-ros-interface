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

import threading

from collections import deque
from std_msgs.msg import Int16



class DataSet(object):
    default_color = QtCore.Qt.black

    def __init__(self, parent):
        self.parent = parent
        self.subscriber = None

        self.enabled = False

        self.points = []
        self.init_dataset()
        self.scaled_color = []
        self.raw_color = []

        self.change_color(Qt.QColor(255,0,0))

    def init_dataset(self):
        tmp = [0]* self.parent.data_points_size
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

        #contains a data set to store the data
        self.data_set = DataSet(self.parent)

        QtGui.QFrame.__init__(self)
        self.layout = QtGui.QHBoxLayout()

        #add a combo box to select the topic
        self.topic_box = QtGui.QComboBox(self)
        self.topic_box.setToolTip("Choose which topic you want to plot.")
        self.topic_box.addItem("None")
        for pub in parent.all_pubs:
            self.topic_box.addItem(pub[0])
        self.connect(self.topic_box, QtCore.SIGNAL('activated(int)'), self.onChanged)
        self.layout.addWidget(self.topic_box)

        #add a button to change the color
        self.change_color_btn = QtGui.QPushButton()
        self.change_color_btn.setIcon(QtGui.QIcon(self.parent.parent.parent.rootPath + '/images/icons/color_wheel.png'))
        self.change_color_btn.setToolTip("Change the color for this topic")
        self.change_color_btn.setFixedWidth(30)
        self.connect(self.change_color_btn, QtCore.SIGNAL('clicked()'), self.change_color_clicked)
        self.layout.addWidget(self.change_color_btn)

        #add a button to add a subscribe topic frame
        self.add_subscribe_topic_btn = QtGui.QPushButton()
        self.add_subscribe_topic_btn.setText('+')
        self.add_subscribe_topic_btn.setToolTip("Add a new topic to plot.")
        self.add_subscribe_topic_btn.setFixedWidth(30)
        self.connect(self.add_subscribe_topic_btn, QtCore.SIGNAL('clicked()'),self.add_subscribe_topic_clicked)
        self.layout.addWidget(self.add_subscribe_topic_btn)

        #add a button to remove the current subscribe topic frame
        if len(self.parent.subscribe_topic_frames) > 0:
            self.remove_topic_btn = QtGui.QPushButton()
            self.remove_topic_btn.setText('-')
            self.remove_topic_btn.setToolTip("Remove this topic.")
            self.remove_topic_btn.setFixedWidth(30)
            self.connect(self.remove_topic_btn, QtCore.SIGNAL('clicked()'),self.remove_topic_clicked)
            self.layout.addWidget(self.remove_topic_btn)

        self.display_last_value = QtGui.QLabel()
        self.layout.addWidget(self.display_last_value)

        #set a color by default
        self.change_color_clicked(const_color = Qt.QColor(255,0,0))

        self.setLayout(self.layout)

    def refresh_topics(self):
        self.topic_box.clear()
        self.topic_box.addItem("None")
        for pub in self.parent.all_pubs:
            self.topic_box.addItem(pub[0])

    def onChanged(self, index):
        text = self.topic_box.currentText()
        #unsubscribe the current topic
        if self.data_set.subscriber != None:
            self.data_set.subscriber.unregister()

        if text != "None":
            self.data_set.enabled = True
            self.data_set.subscriber = rospy.Subscriber(str(text), Int16, self.parent.msg_callback, self.subscriber_index)
        else:
            self.data_set.enabled = False

    def update_display_last_value(self, value):
        txt = ""
        txt += str(value)
        txt += " / "
        txt += str(hex( value ))
        self.display_last_value.setText(txt)


    def change_color_clicked(self, const_color = None):
        if const_color == None:
            col = QtGui.QColorDialog.getColor()

        else:
            col = const_color

        if col.isValid():
            self.data_set.change_color(col)

            html_color = "QLabel { color : "
            html_color += col.name()
            html_color += "; }"
            self.display_last_value.setStyleSheet(html_color)

    def add_subscribe_topic_clicked(self):
        self.parent.add_topic_subscriber()
        Qt.QTimer.singleShot(0, self.adjustSize)

    def remove_topic_clicked(self):
        if self.data_set.subscriber != None:
            self.data_set.subscriber.unregister()
            self.parent.subscriber = None
        self.parent.subscribe_topic_frames.remove(self)

        self.topic_box.setParent(None)
        self.change_color_btn.setParent(None)
        self.add_subscribe_topic_btn.setParent(None)
        self.remove_topic_btn.setParent(None)
        self.display_last_value.setParent(None)

        #refresh the indexes
        for i,sub_frame in enumerate(self.parent.subscribe_topic_frames):
            sub_frame.subscriber_index = i

        Qt.QTimer.singleShot(0, self.adjustSize)
        del self

class SensorScope(OpenGLGenericPlugin):
    """
    Plots some chosen debug values.
    """
    name = "Sensor Scope"

    def __init__(self):
        OpenGLGenericPlugin.__init__(self, self.paint_method)
        self.data_points_size = self.open_gl_widget.number_of_points
        self.all_pubs = None
        self.topic_checker = RosTopicChecker()
        self.refresh_topics()

        self.mutex = threading.Lock()

        self.control_layout = QtGui.QVBoxLayout()

        self.btn_frame = QtGui.QFrame()
        self.btn_frame_layout = QtGui.QHBoxLayout()
        # add a button to play/pause the display
        self.play_btn = QtGui.QPushButton()
        self.play_btn.setFixedWidth(30)
        self.play_btn.setToolTip("Play/Pause the plots")
        self.btn_frame.connect(self.play_btn, QtCore.SIGNAL('clicked()'), self.button_play_clicked)
        self.btn_frame_layout.addWidget(self.play_btn)

        #add a button to refresh the topics
        self.refresh_btn = QtGui.QPushButton()
        self.refresh_btn.setFixedWidth(30)
        self.refresh_btn.setToolTip("Refresh the list of topics")
        self.btn_frame.connect(self.refresh_btn, QtCore.SIGNAL('clicked()'), self.button_refresh_clicked)
        self.btn_frame_layout.addWidget(self.refresh_btn)

        self.btn_frame.setFixedWidth(80)
        self.btn_frame.setFixedHeight(50)
        self.btn_frame.setLayout(self.btn_frame_layout)

        self.control_layout.addWidget(self.btn_frame)
        self.subscribe_topic_frames = []

        self.time_slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.time_slider.setMinimum(0)
        self.time_slider.setMaximum(self.data_points_size - self.open_gl_widget.number_of_points_to_display)
        self.time_slider.setInvertedControls(True)
        self.time_slider.setInvertedAppearance(True)
        self.time_slider.setEnabled(False)
        self.frame.connect(self.time_slider, QtCore.SIGNAL('valueChanged(int)'), self.time_changed)
        self.layout.addWidget(self.time_slider)

        self.paused = False

    def resized(self):
        #we divide by 100 because we want to scroll 100 points per 100 points
        self.time_slider.setMaximum((self.data_points_size - self.open_gl_widget.number_of_points_to_display)/100 - 1)

    def activate(self):
        OpenGLGenericPlugin.activate(self)
        self.play_btn.setIcon(QtGui.QIcon(self.parent.parent.rootPath + '/images/icons/pause.png'))
        self.refresh_btn.setIcon(QtGui.QIcon(self.parent.parent.rootPath + '/images/icons/refresh.png'))

        self.add_topic_subscriber()

        self.control_frame.setLayout(self.control_layout)

    def add_topic_subscriber(self):
        index = len(self.subscribe_topic_frames)
        tmp_stf = SubscribeTopicFrame(self, index)
        self.control_layout.addWidget( tmp_stf )
        self.subscribe_topic_frames.append( tmp_stf )

    def msg_callback(self, msg, index):
        """
        Received a message on the topic. Adding the new point to
        the queue.
        """
        #received message from subscriber at index: update the last
        # point and pop the first point
        #print index, " ", msg.data
        if self.paused:
            return

        self.mutex.acquire()
        for sub_frame in self.subscribe_topic_frames:
            if sub_frame.data_set.enabled:
                if sub_frame.subscriber_index == index:
                    sub_frame.data_set.points.append(msg.data)
                else:
                    sub_frame.data_set.points.append(sub_frame.data_set.points[-1])
            #else:
                # if the data set is not enabled (i.e. subscribing to None),
                # we only draw a line on 0
            #    sub_frame.data_set.points.append(0)
                sub_frame.data_set.points.popleft()
        self.mutex.release()

    def paint_method(self, display_frame = 0):
        '''
        Drawing routine: this function is called periodically.

        @display_frame: the frame to display: we have more points than we display.
                        By default, we display the latest values, but by playing
                        with the time_slider we can then go back in time.
        '''
        if self.paused:
            if display_frame == 0:
                return

        display_points = []
        colors = []

        self.mutex.acquire()
        for sub_frame in self.subscribe_topic_frames:
            #update the value in the label
            sub_frame.update_display_last_value(sub_frame.data_set.points[-1])

            #we want to keep the last point of the raw data in the middle of
            # the screen
            offset = self.open_gl_widget.height/2 - sub_frame.data_set.points[-1]

            for display_index in range(0, self.open_gl_widget.number_of_points_to_display):
                # add the raw data
                colors.append(sub_frame.data_set.get_raw_color())
                data_index = self.display_to_data_index(display_index, display_frame)
                display_points.append([display_index, sub_frame.data_set.points[data_index] + offset])

                #also add the scaled data
                colors.append(sub_frame.data_set.get_scaled_color())
                display_points.append([display_index, self.scale_data(sub_frame.data_set.points[data_index])] )

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

        self.mutex.release()

        glDisableClientState(GL_VERTEX_ARRAY)
        glDisableClientState(GL_COLOR_ARRAY)
        glFlush()
        self.open_gl_widget.update()

    def time_changed(self, value):
        self.paint_method(value)

    def button_play_clicked(self):
        #lock mutex here
        if not self.paused:
            self.paused = True
            self.play_btn.setIcon(QtGui.QIcon(self.parent.parent.rootPath + '/images/icons/play.png'))
            self.time_slider.setEnabled(True)
        else:
            self.paused = False
            self.play_btn.setIcon(QtGui.QIcon(self.parent.parent.rootPath + '/images/icons/pause.png'))
            self.time_slider.setEnabled(False)

    def button_refresh_clicked(self):
        self.refresh_topics()
        for sub_frame in self.subscribe_topic_frames:
            sub_frame.refresh_topics()

    def refresh_topics(self):
        self.all_pubs = self.topic_checker.get_topics()

    def display_to_data_index(self, display_index, display_frame):
        # we multiply display_frame by a 100 because we're scrolling 100 points per 100 points
        data_index = self.data_points_size - self.open_gl_widget.number_of_points_to_display + display_index - (display_frame*100)
        return data_index

    def scale_data(self, data, data_max = 65536):
        scaled_data = (data * self.open_gl_widget.height) / data_max
        return scaled_data

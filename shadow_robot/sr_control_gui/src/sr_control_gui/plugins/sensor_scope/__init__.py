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


    def activate(self):
        OpenGLGenericPlugin.activate(self)

        self.subscribers.append( rospy.Subscriber("/test_1", Int16, self.msg_callback, 0) )
        tmp_dataset1 = DataSet(self.open_gl_widget, index = -1)
        tmp_dataset1.change_color(125,125,125)
        self.datasets.append(tmp_dataset1)

        self.subscribers.append( rospy.Subscriber("/test_2", Int16, self.msg_callback, 1) )
        tmp_dataset2 = DataSet(self.open_gl_widget, index = 1)
        tmp_dataset2.change_color(200,20,20)
        self.datasets.append(tmp_dataset2)


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
        for data_set_id,data_set in enumerate(self.datasets):
            for index in range(0, self.points_size - 1):
                data_set.lines[index].setLine(index, data_set.points[index],
                                              index + 1, data_set.points[index + 1])

if __name__ == "__main__":
    a = SensorScope()

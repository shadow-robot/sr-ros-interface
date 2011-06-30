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

from std_msgs.msg import Int16

class SensorScope(OpenGLGenericPlugin):
    """
    Plots some chosen debug values.
    """
    name = "Sensor Scope"

    def __init__(self):
        OpenGLGenericPlugin.__init__(self, self.paint_method)
        self.subscribers = []

        #1000 points are displayed
        self.points_size = 500
        self.points = [[0,0]]* self.points_size
        self.index_points = 0

    def activate(self):
        OpenGLGenericPlugin.activate(self)

        self.subscribers.append( rospy.Subscriber("/test", Int16, self.msg_callback, 0) )

    def msg_callback(self, msg, index):
        """
        Received a message on the topic. Adding the new point to
        the queue.
        """
        #received message from subscriber at index.
        self.points[self.index_points] = [self.index_points, msg.data]

        self.index_points += 1
        if self.index_points > self.points_size - 1:
            self.index_points = 0


    def paint_method(self):
        '''
        Drawing routine: this function is called periodically.
        '''
        #animations = []
        for data_point, graph_point in zip(self.points, self.open_gl_widget.points):
            graph_point.setPos(data_point[0], data_point[1])
            #animation=QtGui.QGraphicsItemAnimation()
            #duration of the animation
            #timeline=QtCore.QTimeLine(1000/Config.open_gl_generic_plugin_config.refresh_frequency)
            # 5 steps
            #timeline.setFrameRange(0,5)
            #I want that, at time t, the item be at point x,y
            #print " setting pos: ", data_point
            #animation.setPosAt(0,QtCore.QPointF(data_point[0], data_point[1]))

            #It should animate this specific item
            #animation.setItem(graph_point)
            # And the whole animation is this long, and has
            # this many steps as I set in timeline.
            #animation.setTimeLine(timeline)
            #animations.append(animation)

        #[ animation.timeLine().start() for animation in animations ]


if __name__ == "__main__":
    a = SensorScope()

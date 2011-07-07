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

from PyQt4 import QtCore, QtGui, Qt

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PyQt4 import QtGui
from PyQt4.QtOpenGL import *

from generic_plugin import GenericPlugin
from config import *

class GenericGLWidget(QGLWidget):
    """
    A generic openGL frame which is embedded in
    the OpenGLGenericPlugin frame.
    """
    number_of_points = 500000
    number_of_points_to_display = 500
    height = 400

    def __init__(self, parent, paint_method, plugin_parent):
        QGLWidget.__init__(self, parent)

        self.parent = plugin_parent
        self.setMinimumSize(400, 400)
        self.paint_method = paint_method

        self.refresh_timer = QtCore.QTimer()
        QtCore.QObject.connect(self.refresh_timer, QtCore.SIGNAL("timeout()"), self.animate)

    def animate(self):
        '''
        Virtual drawing routine: needs to be overloaded
        '''
        try:
            self.paint_method()
        except:
            pass

    def resizeEvent(self, event):
        w = event.size().width()
        h = event.size().height()

        self.height = h

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glViewport(0, 0, w, h)
        gluOrtho2D(0.0, w, 0.0, h)

        self.number_of_points_to_display = w
        self.parent.resized()

    def initializeGL(self):
        '''
        Initialize GL
        '''
        # set viewing projection
        glClearColor(0.0, 0.0, 0.0, 1.0)
        glClearDepth(1.0)

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glViewport(0, 0, 400, 400)
        gluOrtho2D(0.0, 400.0, 0.0, 400.0)

class OpenGLGenericPlugin(GenericPlugin):
    """
    A generic plugin implement an openGL widget.
    """
    name = "OpenGL Generic Plugin"

    def __init__(self, paint_method):
        GenericPlugin.__init__(self)

        self.splitter = QtGui.QSplitter(QtCore.Qt.Horizontal)

        #a control frame, will contain the buttons / dropdown etc...
        self.control_frame = QtGui.QFrame()
        self.splitter.addWidget(self.control_frame)

        #the visualization frame
        self.layout = QtGui.QVBoxLayout()
        self.frame = QtGui.QFrame()
        self.open_gl_widget = GenericGLWidget(self.frame, paint_method, self)
        self.layout.addWidget(self.open_gl_widget)

        self.frame.setLayout(self.layout)
        self.splitter.addWidget(self.frame)

        self.window.setWidget(self.splitter)

    def activate(self):
        GenericPlugin.activate(self)
        self.open_gl_widget.refresh_timer.start( 1000/Config.open_gl_generic_plugin_config.refresh_frequency )

    def on_close(self):
        self.open_gl_widget.refresh_timer.stop()
        GenericPlugin.on_close(self)

    def depends(self):
        return None

    def resized(self):
        pass

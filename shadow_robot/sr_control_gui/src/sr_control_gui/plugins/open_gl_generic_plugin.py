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
from PyQt4 import QtGui
from PyQt4.QtOpenGL import *

from generic_plugin import GenericPlugin
from config import *

class GenericGLWidget(QtGui.QGraphicsView):
    """
    A generic openGL frame which is embedded in
    the OpenGLGenericPlugin frame.
    """
    number_of_points = 2000
    def __init__(self, parent, paint_method):
        QtGui.QGraphicsView.__init__(self,parent)
        self.scene=QtGui.QGraphicsScene()
        self.setScene(self.scene)
        self.setViewport(QGLWidget())

        self.setMinimumSize(500, 500)
        self.paint_method = paint_method

        self.center_at_the_end()

        self.refresh_timer = QtCore.QTimer()
        QtCore.QObject.connect(self.refresh_timer, QtCore.SIGNAL("timeout()"), self.animate)

    def animate(self):
        '''
        Virtual drawing routine: needs to be overloaded
        '''
        self.paint_method()

        self.update()

    def center_at_the_end(self):
        #center on the further item on the right
        if( len(self.scene.items()) > 0):
            self.centerOn(self.scene.items()[0])

    def resizeEvent(self, event):
        self.center_at_the_end()


class OpenGLGenericPlugin(GenericPlugin):
    """
    A generic plugin implement an openGL widget.
    """
    name = "OpenGL Generic Plugin"

    def __init__(self, paint_method):
        GenericPlugin.__init__(self)

        self.layout = QtGui.QHBoxLayout()
        self.frame = QtGui.QFrame()

        self.open_gl_widget = GenericGLWidget(self.frame, paint_method)
        self.layout.addWidget(self.open_gl_widget)
        self.frame.setLayout(self.layout)
        self.window.setWidget(self.frame)

    def activate(self):
        GenericPlugin.activate(self)
        self.open_gl_widget.refresh_timer.start( 1000/Config.open_gl_generic_plugin_config.refresh_frequency )

    def on_close(self):
        self.open_gl_widget.refresh_timer.stop()
        GenericPlugin.on_close(self)

    def depends(self):
        return None

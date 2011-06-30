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
from OpenGL.GL import *
from OpenGL.GLU import *
from PyQt4 import QtGui
from PyQt4.QtOpenGL import *

from std_msgs.msg import Float64

class SensorScope(OpenGLGenericPlugin):
    """
    Plots some chosen debug values.
    """
    name = "Sensor Scope"

    def __init__(self):
        OpenGLGenericPlugin.__init__(self, self.paint_method)

    def paint_method(self):
        '''
        Drawing routine
        '''

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        # Draw the spiral in 'immediate mode'
        # WARNING: You should not be doing the spiral calculation inside the loop
        # even if you are using glBegin/glEnd, sin/cos are fairly expensive functions
        # I've left it here as is to make the code simpler.
        radius = 1.0
        x = radius*math.sin(0)
        y = radius*math.cos(0)
        glColor(0.0, 1.0, 0.0)
        glBegin(GL_LINE_STRIP)
        for deg in xrange(1000):
            glVertex(x, y, 0.0)
            rad = math.radians(deg)
            radius -= 0.001
            x = radius*math.sin(rad)
            y = radius*math.cos(rad)
        glEnd()

        glEnableClientState(GL_VERTEX_ARRAY)

        spiral_array = []

        # Second Spiral using "array immediate mode" (i.e. Vertex Arrays)
        radius = 0.8
        x = radius*math.sin(0)
        y = radius*math.cos(0)
        glColor(1.0, 0.0, 0.0)
        for deg in xrange(820):
            spiral_array.append([x, y])
            rad = math.radians(deg)
            radius -= 0.001
            x = radius*math.sin(rad)
            y = radius*math.cos(rad)

        glVertexPointerf(spiral_array)
        glDrawArrays(GL_LINE_STRIP, 0, len(spiral_array))


if __name__ == "__main__":
    a = SensorScope()

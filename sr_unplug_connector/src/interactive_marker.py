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

import roslib; roslib.load_manifest('sr_unplug_connector')
import rospy

from interactive_markers.interactive_marker_server import *

class InteractiveConnectorSelector(object):
    def __init__(self, object_names, callback_fct):
        # create an interactive marker server on the topic namespace simple_marker
        self.server = InteractiveMarkerServer("select_connector")

        self.callback_function = callback_fct
        self.object_names = object_names

        self.int_markers = {}
        self.object_markers = {}
        self.object_controls = {}
        self.select_controls = {}

        for object_name in self.object_names:
            self.create_marker( object_name )

        rospy.spin()

    def create_marker(self, object_name):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/"+object_name
        int_marker.name = object_name
        int_marker.description = "Select this object"
        int_marker.pose.position.z = -0.3

        self.int_markers[ object_name ] = int_marker

        # create a marker over the object
        object_marker = Marker()
        object_marker.type = Marker.SPHERE
        object_marker.pose.position.z = 0.6

        object_marker.scale.x = 0.1
        object_marker.scale.y = 0.1
        object_marker.scale.z = 0.1
        object_marker.color.r = 0.6
        object_marker.color.g = 0.02
        object_marker.color.b = 1.0
        object_marker.color.a = 1.0
        self.object_markers[object_name] = object_marker

        # create a non-interactive control which contains the box
        object_control = InteractiveMarkerControl()
        object_control.always_visible = True
        object_control.markers.append( object_marker )

        self.object_controls[object_name] = object_control

        # add the control to the interactive marker
        self.int_markers[object_name].controls.append( object_control )

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        select_control = InteractiveMarkerControl()
        select_control.name = "select_"+object_name
        select_control.description = "______\n|CLICK|"
        select_control.interaction_mode = InteractiveMarkerControl.MENU
        #select_control.menu_entries.push_back()
        self.select_controls[object_name] = select_control

        # add the control to the interactive marker
        self.int_markers[object_name].controls.append(select_control);

        # add the interactive marker to our collection &
        # tell the server to call processFeedback() when feedback arrives for it
        self.server.insert(self.int_markers[object_name], self.processFeedback)

        # 'commit' changes and send to all clients
        self.server.applyChanges()

    def processFeedback(self, feedback):
        selected_name = feedback.marker_name
        for index, name in enumerate(self.object_controls):
            self.object_controls[name].markers.remove( self.object_markers[name] )

            if name == selected_name:
                self.object_markers[name].pose.position.z = 0.65

                self.object_markers[name].scale.x = 0.15
                self.object_markers[name].scale.y = 0.15
                self.object_markers[name].scale.z = 0.15
                self.object_markers[name].color.r = 0.32
                self.object_markers[name].color.g = 0.83
                self.object_markers[name].color.b = 0.15
            else:
                self.object_markers[name].pose.position.z = 0.6

                self.object_markers[name].scale.x = 0.1
                self.object_markers[name].scale.y = 0.1
                self.object_markers[name].scale.z = 0.1
                self.object_markers[name].color.r = 0.6
                self.object_markers[name].color.g = 0.02
                self.object_markers[name].color.b = 1.0

            self.object_controls[name].markers.append( self.object_markers[name] )

            self.server.insert(self.int_markers[name])

        self.server.applyChanges()
        self.callback_function( selected_name )


if __name__=="__main__":
    rospy.init_node("simple_marker")

    int_mark = InteractiveConnectorSelector(["Cup.RobotAreCoolCup", "srh/position/forearm_motor"], None)


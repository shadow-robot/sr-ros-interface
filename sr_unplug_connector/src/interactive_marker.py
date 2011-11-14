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
    def __init__(self, object_name, index_object):
        # create an interactive marker server on the topic namespace simple_marker
        self.server = InteractiveMarkerServer("select_connector")

        self.object_name = object_name

        # create an interactive marker for our server
        self.int_marker = InteractiveMarker()
        self.int_marker.header.frame_id = "/"+object_name
        self.int_marker.name = object_name
        self.int_marker.description = "Select this object"
        self.int_marker.pose.position.z = 0.05


        # create a grey box marker
        self.object_marker = Marker()
        self.object_marker.type = Marker.SPHERE
        self.object_marker.pose.position.z = 0.25

        self.object_marker.scale.x = 0.1
        self.object_marker.scale.y = 0.1
        self.object_marker.scale.z = 0.1
        self.object_marker.color.r = 0.6
        self.object_marker.color.g = 0.02
        self.object_marker.color.b = 1.0
        self.object_marker.color.a = 1.0

        # create a non-interactive control which contains the box
        self.object_control = InteractiveMarkerControl()
        self.object_control.always_visible = True
        self.object_control.markers.append( self.object_marker )

        # add the control to the interactive marker
        self.int_marker.controls.append( self.object_control )

        # create a control which will move the box
        # this control does not contain any markers,
        # which will cause RViz to insert two arrows
        self.select_control = InteractiveMarkerControl()
        self.select_control.name = "select_"+object_name
        self.select_control.description = "Select this object: CLICK ME"
        self.select_control.interaction_mode = InteractiveMarkerControl.MENU
        #select_control.menu_entries.push_back()

        # add the control to the interactive marker
        self.int_marker.controls.append(self.select_control);

        # add the interactive marker to our collection &
        # tell the server to call processFeedback() when feedback arrives for it
        self.server.insert(self.int_marker, self.processFeedback)

        # 'commit' changes and send to all clients
        self.server.applyChanges()

        rospy.spin()

    def processFeedback(self, feedback):
        #p = feedback.menu_entry_id
        self.object_control.markers.remove( self.object_marker )

        self.object_marker.pose.position.z = 0.25

        self.object_marker.scale.x = 0.15
        self.object_marker.scale.y = 0.15
        self.object_marker.scale.z = 0.15
        self.object_marker.color.r = 0.32
        self.object_marker.color.g = 0.83
        self.object_marker.color.b = 0.15
        self.object_marker.color.a = 1.0

        self.object_control.markers.append( self.object_marker )
        print "clicked"

        self.server.insert(self.int_marker)

        self.server.applyChanges()


if __name__=="__main__":
    rospy.init_node("simple_marker")

    int_mark = InteractiveConnectorSelector("Cup.RobotAreCoolCup", 0)


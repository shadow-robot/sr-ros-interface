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
import socket
import roslib.exceptions
import roslib.names
import roslib.scriptutil
import roslib.message
import rosgraph.masterapi

class RosTopicChecker(object):
    def __init__(self):
        pass

    def get_topics(self, topic = None, publishers_only=True):
        master = rosgraph.masterapi.Master('/rostopic')
        try:
            state = master.getSystemState()

            pubs, subs, _ = state
            if topic:
                # filter based on topic
                topic_ns = roslib.names.make_global_ns(topic)        
                subs = (x for x in subs if x[0] == topic or x[0].startswith(topic_ns))
                pubs = (x for x in pubs if x[0] == topic or x[0].startswith(topic_ns))
            
        except socket.error:
            raise ROSTopicIOException("Unable to communicate with master!")

        if publishers_only:
            return pubs
        else:
            return subs, pubs

if __name__ == "__main__":
    test = RosTopicChecker()
    print test.get_topics()


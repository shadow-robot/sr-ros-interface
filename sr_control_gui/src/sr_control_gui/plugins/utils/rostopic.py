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

    def get_topics(self, topic = None, publishers_only=True, topic_filter = "std_msgs/Int16"):
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

        subs, pubs = self.filter_topics(topic_filter, subs, pubs)

        if publishers_only:
            return pubs
        else:
            return subs, pubs

    def filter_topics(self, topic_filter, subs, pubs):
        filtered_sub = []
        filtered_pub = []

        for sub in subs:
            tmp = self._get_topic_type(sub[0])
            if tmp[0] == topic_filter:
                filtered_sub.append(sub)

        for pub in pubs:
            tmp = self._get_topic_type(pub[0])
            if tmp[0] == topic_filter:
                filtered_pub.append(pub)
        return filtered_sub, filtered_pub

    def _master_get_topic_types(self, master):
        try:
            val = master.getTopicTypes()
        except xmlrpclib.Fault:
            print >> sys.stderr, "WARNING: rostopic is being used against an older version of ROS/roscore"
            val = master.getPublishedTopics('/')
        return val

    def _get_topic_type(self, topic):
        """
        subroutine for getting the topic type
        @return: topic type, real topic name and fn to evaluate the message instance
        if the topic points to a field within a topic, e.g. /rosout/msg
        @rtype: str, str, fn
        """
        try:
            val = self._master_get_topic_types(rosgraph.masterapi.Master('/rostopic'))
        except socket.error:
            raise ROSTopicIOException("Unable to communicate with master!")

        # exact match first, followed by prefix match
        matches = [(t, t_type) for t, t_type in val if t == topic]
        if not matches:
            matches = [(t, t_type) for t, t_type in val if topic.startswith(t+'/')]
        if matches:
            #TODO logic for multiple matches if we are prefix matching
            t, t_type = matches[0]
            if t_type == roslib.names.ANYTYPE:
                return None, None, None
            return t_type, t, self.msgevalgen(topic[len(t):])
        else:
            return None, None, None

    def msgevalgen(self, pattern):
        """
        Generates a function that returns the relevant field (aka 'subtopic') of a Message object
        @param pattern: subtopic, e.g. /x. Must have a leading '/' if specified.
        @type  pattern: str
        @return: function that converts a message into the desired value
        @rtype: fn(rospy.Message) -> value
        """
        if not pattern or pattern == '/':
            return None
        def msgeval(msg):
            # I will probably replace this with some less beautiful but more efficient
            try:
                return eval('msg'+'.'.join(pattern.split('/')))
            except AttributeError, e:
                sys.stdout.write("no field named [%s]"%pattern+"\n")
                return None
            return msgeval


if __name__ == "__main__":
    test = RosTopicChecker()
    print test.get_topics()


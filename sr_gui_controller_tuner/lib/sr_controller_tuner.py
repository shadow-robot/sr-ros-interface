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

import roslib
roslib.load_manifest('sr_gui_controller_tuner')
import rospy

from xml.etree import ElementTree as ET
from pr2_mechanism_msgs.srv import ListControllers

class CtrlSettings(object):
    """
    """

    def __init__(self, xml_path, controller_type):
        """
        """
        self.headers = []

        #open and parses the xml config file
        xml_file = open(xml_path)
        xml_tree = ET.parse(xml_file)
        xml_file.close()

        #read the settings from the xml file
        ctrl_tree = None
        for ctrl in xml_tree.findall("controller"):
            ctrl_name = ctrl.attrib['name']
            if ctrl_name == controller_type:
                ctrl_tree = ctrl
                break

        if ctrl_tree == None:
            rospy.logerr("Couldn't find the settings for the controller " + controller_type)

        #read the headers settings
        xml_headers = ctrl.find("headers")
        for header in xml_headers.findall("item"):
            self.headers.append( header.attrib )

        self.nb_columns = len(self.headers)

        self.hand_item = ["Hand"]
        for i in range(0, self.nb_columns - 1):
            self.hand_item.append("")

        #read the fingers and the motors from the xml file
        self.fingers = []
        self.motors = []
        all_fingers = xml_tree.find("fingers")
        for finger in all_fingers.findall("finger"):
            finger_row = [ finger.attrib['name'] ]
            for i in range(0, self.nb_columns - 1):
                finger_row.append("")
            self.fingers.append( finger_row )

            motors_for_finger = []
            for motor in finger.findall("motor"):
                motor_row = [ "", motor.attrib['name'] ]
                for i in range(0, self.nb_columns - 2):
                    motor_row.append("")
                motors_for_finger.append( motor_row )

            self.motors.append( motors_for_finger )


class SrControllerTunerLib(object):
    """
    """

    def __init__(self, xml_path):
        """
        """
        self.xml_path = xml_path
        self.all_controller_types = ["Motor Force", "Position", "Velocity",
                                     "Mixed Position/Velocity", "Effort"]

    def get_ctrls(self):
        return ["Motor Force", "Position"]

        running_ctrls = []

        rospy.wait_for_service('/pr2_controller_manager/list_controllers')
        controllers = rospy.ServiceProxy('/pr2_controller_manager/list_controllers', ListControllers)
        resp = None
        try:
            resp = controllers()
        except rospy.ServiceException, e:
            print "Service did not process request: %s"%str(e)

        running_ctrls.append("Motor Force")
        if resp != None:
            for state,controller in zip(resp.state, resp.controllers):
                if state == "running":
                    split = controller.split("_")
                    ctrl_type_tmp = split[2]
                    for defined_ctrl_type in self.all_controller_types:
                        if ctrl_type_tmp.lower() in defined_ctrl_type.lower():
                            running_ctrls.append(defined_ctrl_type)
                            return running_ctrls

    def get_controller_settings( self, controller_type ):
        """
        Parses a file containing the controller settings
        and their min and max values, and return them.
        """
        ctrl_settings = CtrlSettings(self.xml_path, controller_type)

        return ctrl_settings

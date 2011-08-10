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

from config import Config
from PyQt4 import QtCore, QtGui, Qt

from generic_plugin import GenericPlugin
from std_msgs.msg import Float64

from etherCAT_hand_lib import EtherCAT_Hand_Lib

class HandCalibration(object):
    """
    Calibration procedure for the etherCAT hand.
    """

    def __init__(self, ):
        """
        Calibration procedure for the etherCAT hand.
        """
        self.calibration_map = {}
        self.robot_lib = EtherCAT_Hand_Lib()

    def activate(self):
        self.robot_lib.activate()

    def on_close(self):
        self.robot_lib.on_close()

    def calibrate(self, joint_name, calibrated_value):
        raw_value = self.robot_lib.get_raw_value(joint_name)

        if joint_name not in self.calibration_map.keys():
            self.calibration_map[joint_name] = []
        self.calibration_map[joint_name].append([raw_value, calibrated_value])

        return raw_value

    def write_calibration(self, path):
        pass

class HandCalibrationPlugin(GenericPlugin):
    """
    Plugin for the calibration procedure for the etherCAT hand.
    """
    name = "Hand Calibration"

    joint_map = {"First Finger":[ [ "FFJ1", [ ["0.0", "0.0"],
                                              ["0.0", "90.0"] ] ],
                                  [ "FFJ2", [ ["0.0", "0.0"],
                                              ["0.0", "90.0"] ] ],
                                  [ "FFJ3", [ ["0.0", "0.0"],
                                              ["0.0", "90.0"] ] ],
                                  [ "FFJ4", [ ["0.0", "-25.0"],
                                              ["0.0", "25.0"] ] ] ],

                 "Middle Finger":[ [ "MFJ1", [ ["0.0", "0.0"],
                                               ["0.0", "90.0"] ] ],
                                   [ "MFJ2", [ ["0.0", "0.0"],
                                               ["0.0", "90.0"] ] ],
                                   [ "MFJ3", [ ["0.0", "0.0"],
                                               ["0.0", "90.0"] ] ],
                                   [ "MFJ4", [ ["0.0", "-25.0"],
                                               ["0.0", "25.0"] ] ] ],

                 "Ring Finger":[ [ "RFJ1", [ ["0.0", "0.0"],
                                             ["0.0", "90.0"] ] ],
                                 [ "RFJ2", [ ["0.0", "0.0"],
                                             ["0.0", "90.0"] ] ],
                                 [ "RFJ3", [ ["0.0", "0.0"],
                                             ["0.0", "90.0"] ] ],
                                 [ "RFJ4", [ ["0.0", "-25.0"],
                                             ["0.0", "25.0"] ] ] ]
                 }

    def __init__(self, ):
        """
        Plugin for the calibration procedure for the etherCAT hand.
        """
        GenericPlugin.__init__(self)

        self.hand_calibration = HandCalibration()

        self.frame = QtGui.QFrame()
        self.layout = QtGui.QHBoxLayout()

        self.tree_widget = QtGui.QTreeWidget()
        self.tree_widget.setColumnCount(4)
        self.tree_widget.setHeaderLabels(["Finger", "Joint", "Raw Value", "Calibrated Value"])

        #fill the tree with the values
        for finger in self.joint_map.items():
            finger_item = QtGui.QTreeWidgetItem([finger[0], "", "", ""])
            self.tree_widget.addTopLevelItem(finger_item)

            for joint in finger[1]:
                joint_item = QtGui.QTreeWidgetItem(finger_item, ["", joint[0], "", ""])
                for value in joint[1]:
                    value_item = QtGui.QTreeWidgetItem(joint_item, ["", "", str(value[0]), str(value[1])])
                    self.tree_widget.addTopLevelItem(value_item)
                joint_item.setExpanded(True)
            finger_item.setExpanded(True)

        #calibrate when double clicking on an item
        self.frame.connect(self.tree_widget, QtCore.SIGNAL('itemDoubleClicked (QTreeWidgetItem *, int)'),
                           self.calibrate_item)

        self.layout.addWidget(self.tree_widget)
        self.frame.setLayout(self.layout)
        self.window.setWidget(self.frame)
        self.window.setGeometry(0,0,650,400)

    def activate(self):
        GenericPlugin.activate(self)

        self.hand_calibration.activate()

    def on_close(self):
        GenericPlugin.on_close(self)

        self.hand_calibration.on_close()

    def calibrate_item(self, item, value):
        #Check if it's not a top level item
        if item.parent() is None:
            return

        joint_name = str(item.parent().data(1,0).toString())
        if joint_name is not "":
            calibrated_value = float( item.data(3,0).toString() )
            raw_value = self.hand_calibration.calibrate( joint_name, calibrated_value )

            item.setData(2, 0, raw_value)

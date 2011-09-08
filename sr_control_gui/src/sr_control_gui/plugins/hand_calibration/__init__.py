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
from functools import partial

from config import Config
from PyQt4 import QtCore, QtGui, Qt

from generic_plugin import GenericPlugin
from std_msgs.msg import Float64

from etherCAT_hand_lib import EtherCAT_Hand_Lib
from hand_calibration.calibration_saver import CalibrationSaver

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
        raw_value = self.robot_lib.get_average_raw_value(joint_name, 100)

        if joint_name not in self.calibration_map.keys():
            self.calibration_map[joint_name] = []
        self.calibration_map[joint_name].append([raw_value, calibrated_value])

        return raw_value

    def write_calibration(self, path):
        """
        Generates the yaml configuration file from the
        calibration map and writes it to the specified
        path, using the calibration saver (updates only
        the recalibrated joints).
        """
        calibration_saver = CalibrationSaver(path)

        calibration_to_save = []
        for item in self.calibration_map.items():
            calibration_to_save.append([[item[0]], item[1]])

        calibration_saver.save_settings(calibration_to_save)


class HandCalibrationPlugin(GenericPlugin):
    """
    Plugin for the calibration procedure for the etherCAT hand.
    """
    name = "Hand Calibration"

    joint_map = {"First Finger":[ [ "FFJ1", [ ["0.0", "0.0"],
                                              ["0.0", "22.5"],
                                              ["0.0", "45.0"],
                                              ["0.0", "67.5"],
                                              ["0.0", "90.0"] ] ],
                                  [ "FFJ2", [ ["0.0", "0.0"],
                                              ["0.0", "22.5"],
                                              ["0.0", "45.0"],
                                              ["0.0", "67.5"],
                                              ["0.0", "90.0"] ] ],
                                  [ "FFJ3", [ ["0.0", "0.0"],
                                              ["0.0", "22.5"],
                                              ["0.0", "45.0"],
                                              ["0.0", "67.5"],
                                              ["0.0", "90.0"] ] ],
                                  [ "FFJ4", [ ["0.0", "-25.0"],
                                              ["0.0", "0.0"],
                                              ["0.0", "25.0"] ] ] ],

                 "Middle Finger":[ [ "MFJ1", [ ["0.0", "0.0"],
                                               ["0.0", "22.5"],
                                               ["0.0", "45.0"],
                                               ["0.0", "67.5"],
                                               ["0.0", "90.0"] ] ],
                                   [ "MFJ2", [ ["0.0", "0.0"],
                                               ["0.0", "22.5"],
                                               ["0.0", "45.0"],
                                               ["0.0", "67.5"],
                                               ["0.0", "90.0"] ] ],
                                   [ "MFJ3", [ ["0.0", "0.0"],
                                               ["0.0", "22.5"],
                                               ["0.0", "45.0"],
                                               ["0.0", "67.5"],
                                               ["0.0", "90.0"] ] ],
                                   [ "MFJ4", [ ["0.0", "-25.0"],
                                               ["0.0", "0.0"],
                                               ["0.0", "25.0"] ] ] ],

                 "Ring Finger":[ [ "RFJ1", [ ["0.0", "0.0"],
                                             ["0.0", "22.5"],
                                             ["0.0", "45.0"],
                                             ["0.0", "67.5"],
                                             ["0.0", "90.0"] ] ],
                                 [ "RFJ2", [ ["0.0", "0.0"],
                                             ["0.0", "22.5"],
                                             ["0.0", "45.0"],
                                             ["0.0", "67.5"],
                                             ["0.0", "90.0"] ] ],
                                 [ "RFJ3", [ ["0.0", "0.0"],
                                             ["0.0", "22.5"],
                                             ["0.0", "45.0"],
                                             ["0.0", "67.5"],
                                             ["0.0", "90.0"] ] ],
                                 [ "RFJ4", [ ["0.0", "-25.0"],
                                             ["0.0", "0.0"],
                                             ["0.0", "25.0"] ] ] ],

                 "Little Finger":[ [ "LFJ1", [ ["0.0", "0.0"],
                                               ["0.0", "22.5"],
                                               ["0.0", "45.0"],
                                               ["0.0", "67.5"],
                                               ["0.0", "90.0"] ] ],
                                   [ "LFJ2", [ ["0.0", "0.0"],
                                               ["0.0", "22.5"],
                                               ["0.0", "45.0"],
                                               ["0.0", "67.5"],
                                               ["0.0", "90.0"] ] ],
                                   [ "LFJ3", [ ["0.0", "0.0"],
                                               ["0.0", "22.5"],
                                               ["0.0", "45.0"],
                                               ["0.0", "67.5"],
                                               ["0.0", "90.0"] ] ],
                                   [ "LFJ4", [ ["0.0", "-25.0"],
                                               ["0.0", "0.0"],
                                               ["0.0", "25.0"] ] ] ,
                                   [ "LFJ5", [  ["0.0", "0.0"],
                                               ["0.0", "22.5"],
                                               ["0.0", "45.0"],
                                               ["0.0", "67.5"],
                                               ["0.0", "90.0"] ] ] ],

                 "Thumb":[ [ "THJ1", [ ["0.0", "0.0"],
                                       ["0.0", "22.5"],
                                       ["0.0", "45.0"],
                                       ["0.0", "67.5"],
                                       ["0.0", "90.0"] ] ],
                           [ "THJ2", [ ["0.0", "-30.0"],
                                       ["0.0", "-15.0"],
                                       ["0.0", "0.0"],
                                       ["0.0", "15.0"],
                                       ["0.0", "30.0"] ] ],
                           [ "THJ3", [ ["0.0", "-15.0"],
                                       ["0.0", "0.0"],
                                       ["0.0", "15.0"] ] ],
                           [ "THJ4", [ ["0.0", "0.0"],
                                       ["0.0", "22.5"],
                                       ["0.0", "45.0"],
                                       ["0.0", "67.5"] ] ],
                           [ "THJ5", [  ["0.0", "-60.0"],
                                        ["0.0", "-30.0"],
                                        ["0.0", "0.0"],
                                        ["0.0", "30.0"],
                                        ["0.0", "60.0"] ] ] ],

                 "Wrist":[ [ "WRJ1", [ ["0.0", "-45.0"],
                                       ["0.0", "-22.5"],
                                       ["0.0", "0.0"],
                                       ["0.0", "22.5"],
                                       ["0.0", "35"] ] ],
                           [ "WRJ2", [ ["0.0", "-30.0"],
                                       ["0.0", "0.0"],
                                       ["0.0", "10.0"] ] ] ]
                 }

    green = QtGui.QColor(153, 231, 96)
    red = QtGui.QColor(236, 178, 178)

    indexes_to_calibrate = []
    calibrated_indexes   = []

    def __init__(self, ):
        """
        Plugin for the calibration procedure for the etherCAT hand.
        """
        GenericPlugin.__init__(self)

        self.hand_calibration = HandCalibration()

        self.frame = QtGui.QFrame()
        self.layout = QtGui.QVBoxLayout()

        self.tree_widget = QtGui.QTreeWidget()
        self.tree_widget.setColumnCount(4)
        self.tree_widget.setHeaderLabels(["Finger", "Joint", "Raw Value", "Calibrated Value"])

        #keep the joint items in a vector to be able to
        # display the current joint position
        self.joint_items = []

        self.hand_item = QtGui.QTreeWidgetItem(["Hand", "", "", ""])
        self.tree_widget.addTopLevelItem(self.hand_item)
        #fill the tree with the values
        index_to_calibrate = 0
        for finger in self.joint_map.items():
            finger_item = QtGui.QTreeWidgetItem(self.hand_item, [finger[0], "", "", ""])
            self.tree_widget.addTopLevelItem(finger_item)
            index_to_calibrate += 1

            for joint in finger[1]:
                joint_item = QtGui.QTreeWidgetItem(finger_item, ["", joint[0], "", ""])
                self.joint_items.append(joint_item)
                index_to_calibrate += 1
                for value in joint[1]:
                    value_item = QtGui.QTreeWidgetItem(joint_item, ["", "", str(value[0]), str(value[1])])
                    value_item.setBackgroundColor(0, QtGui.QColor(self.red))

                    self.tree_widget.addTopLevelItem(value_item)
                    index_to_calibrate += 1
                    self.indexes_to_calibrate.append(index_to_calibrate)

                joint_item.setExpanded(True)
            finger_item.setExpanded(True)

        self.hand_item.setExpanded(True)

        #calibrate when double clicking on an item
        self.frame.connect(self.tree_widget, QtCore.SIGNAL('itemDoubleClicked (QTreeWidgetItem *, int)'),
                           self.calibrate_item)

        self.write_calibration_btn = QtGui.QPushButton()
        self.write_calibration_btn.setText("Save INCOMPLETE Calibration")
        self.write_calibration_btn.clicked.connect(partial(self.write_calibration))

        self.all_calibrated = False

        self.layout.addWidget(self.write_calibration_btn)

        self.layout.addWidget(self.tree_widget)
        self.frame.setLayout(self.layout)
        self.window.setWidget(self.frame)
        self.window.setGeometry(0,0,650,400)

        #adjust the column sizes
        for i in range(0,4):
            self.tree_widget.resizeColumnToContents(i)

        #display the current joint position in the GUI
        self.timer = Qt.QTimer()
        self.frame.connect(self.timer, QtCore.SIGNAL('timeout()'), self.update_joint_pos)

    def update_joint_pos(self):
        for joint_item in self.joint_items:
            current_value = self.hand_calibration.robot_lib.get_raw_value( str( joint_item.text(1) ) )
            joint_item.setText( 2, str(current_value) )

    def activate(self):
        GenericPlugin.activate(self)

        self.hand_calibration.activate()
        self.timer.start(200)

    def on_close(self):
        GenericPlugin.on_close(self)

        self.hand_calibration.on_close()
        self.timer.stop()

    def calibrate_item(self, item, index):
        #Check if it's not a top level item
        if item.parent() is None:
            return

        joint_name = str(item.parent().data(1,0).toString())
        if joint_name is not "":
            calibrated_value = float( item.data(3,0).toString() )
            raw_value = self.hand_calibration.calibrate( joint_name, calibrated_value )

            item.setData(2, 0, raw_value)
            item.setBackgroundColor(0, QtGui.QColor(self.green))

            row_index = self.compute_item_index(item)
            self.all_calibrated = self.is_all_calibrated(row_index)

            if self.all_calibrated:
                self.write_calibration_btn.setText("Save Calibration")
            else:
                self.write_calibration_btn.setText("Save INCOMPLETE Calibration")


    def compute_item_index(self, item):
        index = 0
        it = QtGui.QTreeWidgetItemIterator(self.tree_widget)
        while (it.value()):
            if item == it.value():
                return index

            index += 1
            it += 1
        return - 1

    def is_all_calibrated(self, index):
        self.calibrated_indexes.append(index)

        for index_to_calibrate in self.indexes_to_calibrate:
            if index_to_calibrate not in self.calibrated_indexes:
                return False
        return True


    def write_calibration(self):
        filename = QtGui.QFileDialog.getOpenFileName(self.frame, Qt.QString("Write Calibration To"), Qt.QString(""), Qt.QString("*.yaml") )
        self.hand_calibration.write_calibration(filename)

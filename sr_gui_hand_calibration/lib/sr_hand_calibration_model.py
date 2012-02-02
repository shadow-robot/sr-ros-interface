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
roslib.load_manifest('sr_gui_hand_calibration')
import rospy

from QtGui import QTreeWidgetItem, QColor

green = QColor(153, 231, 96)
red = QColor(236, 178, 178)

class IndividualCalibration(object):
    """

    """

    def __init__(self, raw_value = 0, calibrated_value = 0.0,
                 parent_widget = None, tree_widget = None):
        """
        """
        self.raw_value = raw_value
        self.calibrated_value = calibrated_value

        self.widget = QTreeWidgetItem(parent_widget, ["", "", str(self.raw_value), str(self.calibrated_value)])
        for col in xrange(tree_widget.columnCount()):
            self.widget.setBackgroundColor(col, QColor(red))

        tree_widget.addTopLevelItem( self.widget )

class JointCalibration(object):
    """
    """

    def __init__(self, joint_name = "FFJ0",
                 calibrations = [ [0.0, 0.0],
                                  [0.0, 22.5],
                                  [0.0, 45.0],
                                  [0.0, 67.5],
                                  [0.0, 90.0] ],
                 parent_widget = None, tree_widget = None):
        """
        """

        self.calibrations = []

        self.widget = QTreeWidgetItem(parent_widget, ["", joint_name, "", ""])

        for calibration in calibrations:
            self.calibrations.append( IndividualCalibration(calibration[0], calibration[1],
                                                            self.widget, tree_widget) )

        tree_widget.addTopLevelItem(self.widget)

class FingerCalibration(object):
    """
    """

    def __init__(self, finger_name,
                 finger_joints,
                 parent_widget, tree_widget):
        """
        """

        self.widget = QTreeWidgetItem(parent_widget, [finger_name, "", "", ""])

        self.joints = []
        for joint in finger_joints:
            self.joints.append( JointCalibration( joint_name = joint[0],
                                                  calibrations = joint[1],
                                                  parent_widget = self.widget,
                                                  tree_widget = tree_widget ) )

        tree_widget.addTopLevelItem(self.widget)


class HandCalibration(object):
    """
    """
    #TODO: Import this from an xml file?
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

    def __init__(self, fingers = ["First Finger", "Middle Finger",
                                  "Ring Finger", "Little Finger",
                                  "Thumb", "Wrist"],
                 tree_widget = None):
        """
        """
        self.fingers = []

        self.widget = QTreeWidgetItem(["Hand", "", "", ""])

        for finger in fingers:
            if finger in self.joint_map.keys():
                self.fingers.append( FingerCalibration( finger,
                                                        self.joint_map[finger],
                                                        self.widget, tree_widget ) )

            else:
                print finger, " not found in the calibration map"


        self.tree_widget = tree_widget
        self.tree_widget.addTopLevelItem(self.widget)
        self.tree_widget.itemActivated.connect(self.calibrate_item)

    def calibrate_item(self, item):
        for col in xrange(self.tree_widget.columnCount()):
            #calibrate only the calibration lines, not the items for
            # the fingers / joints / hand
            if item.text(2) != "":
                item.setBackgroundColor(col, QColor(green))

        #select the next row by default
        self.tree_widget.setItemSelected( item, False )
        next_item = self.tree_widget.itemBelow(item)
        self.tree_widget.setItemSelected( next_item, True )
        self.tree_widget.setCurrentItem( next_item )


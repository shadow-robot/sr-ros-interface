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

from etherCAT_hand_lib import EtherCAT_Hand_Lib
from QtGui import QTreeWidgetItem, QTreeWidgetItemIterator, QColor, QIcon
from PyQt4.Qt import QTimer
from PyQt4.QtCore import SIGNAL

import yaml
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper
import os

from collections import deque

green = QColor(153, 231, 96)
red = QColor(236, 178, 178)

class IndividualCalibration( QTreeWidgetItem ):
    """
    """

    def __init__(self, joint_name,
                 raw_value, calibrated_value,
                 parent_widget, tree_widget,
                 robot_lib):
        """
        """
        self.joint_name = joint_name
        self.raw_value = raw_value
        self.calibrated_value = calibrated_value
        self.tree_widget = tree_widget
        self.robot_lib = robot_lib

        QTreeWidgetItem.__init__(self, parent_widget, ["", "", str(self.raw_value), str(self.calibrated_value)])

        for col in xrange(self.tree_widget.columnCount()):
            self.setBackgroundColor(col, QColor(red))

        self.tree_widget.addTopLevelItem( self )

        self.is_calibrated = False

    def calibrate(self):
        self.raw_value = 5#self.robot_lib.get_average_raw_value(self.joint_name, 100)
        self.setText( 3, str(self.raw_value) )

        for col in xrange(self.tree_widget.columnCount()):
            #calibrate only the calibration lines, not the items for
            # the fingers / joints / hand
            if self.text(2) != "":
                self.setBackgroundColor(col, QColor(green))

        self.is_calibrated = True


class JointCalibration( QTreeWidgetItem ):
    """
    """

    nb_values_to_check = 5
    def __init__(self, joint_name,
                 calibrations,
                 parent_widget, tree_widget,
                 robot_lib):
        """
        """
        self.joint_name = joint_name
        self.robot_lib = robot_lib

        self.calibrations = []

        self.last_raw_values = deque()

        QTreeWidgetItem.__init__(self, parent_widget, ["", joint_name, "", ""] )

        for calibration in calibrations:
            self.calibrations.append( IndividualCalibration(joint_name,
                                                            calibration[0], calibration[1],
                                                            self, tree_widget, robot_lib) )

        #display the current joint position in the GUI
        self.timer = QTimer()
        self.timer.start(200)
        tree_widget.addTopLevelItem(self)
        tree_widget.connect(self.timer, SIGNAL('timeout()'), self.update_joint_pos)

    def load_joint_calibration(self, calibrations):
        for calibration in calibrations:
            self.calibrations.modify()

    def update_joint_pos(self):
        raw_value = 5#self.robot_lib.get_raw_value( self.joint_name )
        self.setText( 2, str(raw_value) )

        #if the 5 last values are equal, then display a warning
        # as there's always some noise on the values
        self.last_raw_values.append(raw_value)
        if len(self.last_raw_values) > self.nb_values_to_check:
            self.last_raw_values.popleft()

            #only check if we have enough values
            all_equal = True
            last_data = self.last_raw_values[0]
            for data in self.last_raw_values:
                if data != last_data:
                    all_equal = False
                    break
            if all_equal == True:
                self.setIcon(0, QIcon( os.path.join(os.path.dirname(os.path.realpath(__file__)), '../icons/warn.gif')))
                self.setToolTip(0,"No noise on the data for the last " + str(self.nb_values_to_check) + " values, there could be a problem with the sensor.")
            else:
                self.setIcon(0, None)
                self.setToolTip(0, "")




class FingerCalibration( QTreeWidgetItem ):
    """
    """

    def __init__(self, finger_name,
                 finger_joints,
                 parent_widget, tree_widget,
                 robot_lib):
        """
        """

        QTreeWidgetItem.__init__(self, parent_widget, [finger_name, "", "", ""] )

        self.joints = []
        for joint in finger_joints:
            self.joints.append( JointCalibration( joint_name = joint[0],
                                                  calibrations = joint[1],
                                                  parent_widget = self,
                                                  tree_widget = tree_widget,
                                                  robot_lib = robot_lib) )

        tree_widget.addTopLevelItem(self)

class HandCalibration( QTreeWidgetItem ):
    """
    """
    #TODO: Import this from an xml file?
    joint_map = {"First Finger":[ [ "FFJ1", [ [0.0, 0.0],
                                              [0.0, 22.5],
                                              [0.0, 45.0],
                                              [0.0, 67.5],
                                              [0.0, 90.0] ] ],
                                  [ "FFJ2", [ [0.0, 0.0],
                                              [0.0, 22.5],
                                              [0.0, 45.0],
                                              [0.0, 67.5],
                                              [0.0, 90.0] ] ],
                                  [ "FFJ3", [ [0.0, 0.0],
                                              [0.0, 22.5],
                                              [0.0, 45.0],
                                              [0.0, 67.5],
                                              [0.0, 90.0] ] ],
                                  [ "FFJ4", [ [0.0, -25.0],
                                              [0.0, 0.0],
                                              [0.0, 25.0] ] ] ],

                 "Middle Finger":[ [ "MFJ1", [ [0.0, 0.0],
                                               [0.0, 22.5],
                                               [0.0, 45.0],
                                               [0.0, 67.5],
                                               [0.0, 90.0] ] ],
                                   [ "MFJ2", [ [0.0, 0.0],
                                               [0.0, 22.5],
                                               [0.0, 45.0],
                                               [0.0, 67.5],
                                               [0.0, 90.0] ] ],
                                   [ "MFJ3", [ [0.0, 0.0],
                                               [0.0, 22.5],
                                               [0.0, 45.0],
                                               [0.0, 67.5],
                                               [0.0, 90.0] ] ],
                                   [ "MFJ4", [ [0.0, -25.0],
                                               [0.0, 0.0],
                                               [0.0, 25.0] ] ] ],

                 "Ring Finger":[ [ "RFJ1", [ [0.0, 0.0],
                                             [0.0, 22.5],
                                             [0.0, 45.0],
                                             [0.0, 67.5],
                                             [0.0, 90.0] ] ],
                                 [ "RFJ2", [ [0.0, 0.0],
                                             [0.0, 22.5],
                                             [0.0, 45.0],
                                             [0.0, 67.5],
                                             [0.0, 90.0] ] ],
                                 [ "RFJ3", [ [0.0, 0.0],
                                             [0.0, 22.5],
                                             [0.0, 45.0],
                                             [0.0, 67.5],
                                             [0.0, 90.0] ] ],
                                 [ "RFJ4", [ [0.0, -25.0],
                                             [0.0, 0.0],
                                             [0.0, 25.0] ] ] ],

                 "Little Finger":[ [ "LFJ1", [ [0.0, 0.0],
                                               [0.0, 22.5],
                                               [0.0, 45.0],
                                               [0.0, 67.5],
                                               [0.0, 90.0] ] ],
                                   [ "LFJ2", [ [0.0, 0.0],
                                               [0.0, 22.5],
                                               [0.0, 45.0],
                                               [0.0, 67.5],
                                               [0.0, 90.0] ] ],
                                   [ "LFJ3", [ [0.0, 0.0],
                                               [0.0, 22.5],
                                               [0.0, 45.0],
                                               [0.0, 67.5],
                                               [0.0, 90.0] ] ],
                                   [ "LFJ4", [ [0.0, -25.0],
                                               [0.0, 0.0],
                                               [0.0, 25.0] ] ] ,
                                   [ "LFJ5", [ [0.0, 0.0],
                                               [0.0, 22.5],
                                               [0.0, 45.0],
                                               [0.0, 67.5],
                                               [0.0, 90.0] ] ] ],

                 "Thumb":[ [ "THJ1", [ [0.0, 0.0],
                                       [0.0, 22.5],
                                       [0.0, 45.0],
                                       [0.0, 67.5],
                                       [0.0, 90.0] ] ],
                           [ "THJ2", [ [0.0, -30.0],
                                       [0.0, -15.0],
                                       [0.0, 0.0],
                                       [0.0, 15.0],
                                       [0.0, 30.0] ] ],
                           [ "THJ3", [ [0.0, -15.0],
                                       [0.0, 0.0],
                                       [0.0, 15.0] ] ],
                           [ "THJ4", [ [0.0, 0.0],
                                       [0.0, 22.5],
                                       [0.0, 45.0],
                                       [0.0, 67.5] ] ],
                           [ "THJ5", [ [0.0, -60.0],
                                       [0.0, -30.0],
                                       [0.0, 0.0],
                                       [0.0, 30.0],
                                       [0.0, 60.0] ] ] ],

                 "Wrist":[ [ "WRJ1", [ [0.0, -45.0],
                                       [0.0, -22.5],
                                       [0.0, 0.0],
                                       [0.0, 22.5],
                                       [0.0, 35.0] ] ],
                           [ "WRJ2", [ [0.0, -30.0],
                                       [0.0, 0.0],
                                       [0.0, 10.0] ] ] ]
                 }

    def __init__(self,
                 tree_widget,
                 progress_bar,
                 fingers = ["First Finger", "Middle Finger",
                            "Ring Finger", "Little Finger",
                            "Thumb", "Wrist"]):
        """
        """
        self.fingers = []

        QTreeWidgetItem.__init__(self, ["Hand", "", "", ""] )

        self.robot_lib = None#EtherCAT_Hand_Lib()
        #self.robot_lib.activate()

        for finger in fingers:
            if finger in self.joint_map.keys():
                self.fingers.append( FingerCalibration( finger,
                                                        self.joint_map[finger],
                                                        self, tree_widget,
                                                        self.robot_lib) )

            else:
                print finger, " not found in the calibration map"

        self.joint_0_calibration_index = 0

        self.progress_bar = progress_bar

        self.tree_widget = tree_widget
        self.tree_widget.addTopLevelItem(self)
        self.tree_widget.itemActivated.connect(self.calibrate_item)

    def unregister(self):
        self.robot_lib.on_close()

    def calibrate_item(self, item):
        try:
            #only the IndividualCalibration have the calibrate method
            item.calibrate()
        except:
            pass

        self.progress()

        #select the next row by default
        self.tree_widget.setItemSelected( item, False )
        next_item = self.tree_widget.itemBelow(item)
        self.tree_widget.setItemSelected( next_item, True )
        self.tree_widget.setCurrentItem( next_item )

    def calibrate_joint0s(self, btn_joint_0s):
        joint0s = [ "FFJ1", "FFJ2",
                    "MFJ1", "MFJ2",
                    "RFJ1", "RFJ2",
                    "LFJ1", "LFJ2" ]

        it = QTreeWidgetItemIterator( self )
        while it.value():
            if it.value().text(1) in joint0s:
                it += self.joint_0_calibration_index + 1
                it.value().calibrate()
            it += 1

        self.joint_0_calibration_index += 1
        if self.joint_0_calibration_index == len( self.joint_map["First Finger"][0][1] ):
            self.joint_0_calibration_index = 0

        #updating the btn text
        btn_joint_0s.setText( "Save all Joint 0s (angle = "+ str(self.joint_map["First Finger"][0][1][self.joint_0_calibration_index][1]) +")" )

        self.progress()

    def progress(self):
        it = QTreeWidgetItemIterator( self )
        nb_of_items = 0
        nb_of_calibrated_items = 0
        while it.value():
            it += 1
            try:
                if it.value().is_calibrated:
                    nb_of_calibrated_items += 1
                nb_of_items += 1
            except:
                pass

        self.progress_bar.setValue( int( float(nb_of_calibrated_items) / float(nb_of_items) * 100.0 ) )

    def load(self, filepath):
        f = open(filepath,'r')
        document = ""
        for line in f.readlines():
            document += line
        f.close()
        yaml_config = yaml.load(document)
        print yaml_config

        for joint in yaml_config["sr_calibrations"]:
            it = QTreeWidgetItemIterator( self )
            while it.value():
                if it.value().text(1) == joint[0]:
                    print joint

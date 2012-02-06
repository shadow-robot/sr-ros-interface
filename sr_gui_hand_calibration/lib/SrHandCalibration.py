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

from __future__ import division
import os

from rosgui.QtBindingHelper import loadUi
from QtCore import QEvent, QObject, Qt, QTimer, Slot
from QtGui import QDockWidget, QShortcut, QColor, QTreeWidgetItem, QFileDialog, QMessageBox
from QtCore import QVariant

import roslib
roslib.load_manifest('sr_gui_hand_calibration')
import rospy

from sr_hand_calibration_model import HandCalibration

class SrHandCalibration(QObject):

    def __init__(self, parent, plugin_context):
        super(SrHandCalibration, self).__init__(parent)
        self.setObjectName('SrHandCalibration')

        self._publisher = None
        main_window = plugin_context.main_window()
        self._widget = QDockWidget(main_window)

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../uis/SrHandCalibration.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrHandCalibrationUi')
        if plugin_context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % plugin_context.serial_number()))
        main_window.addDockWidget(Qt.RightDockWidgetArea, self._widget)

        # trigger deleteLater for plugin when _widget is closed
        self._widget.installEventFilter(self)

        self._widget.tree_calibration.setColumnCount(4)
        self._widget.tree_calibration.setHeaderLabels(["Finger", "Joint", "Raw Value", "Calibrated Value"])

        self.hand_model = None

        self._widget.btn_save.clicked.connect(self.btn_save_clicked_)
        self._widget.btn_load.clicked.connect(self.btn_load_clicked_)
        self._widget.btn_joint_0s.clicked.connect(self.btn_joint_0s_clicked_)

        self.populate_tree()

    @Slot(str)

    def populate_tree(self):
        self._widget.tree_calibration.clear()

        self.hand_model = HandCalibration( tree_widget = self._widget.tree_calibration, progress_bar = self._widget.progress )

        self._widget.tree_calibration.expandAll()

        for col in range(0, self._widget.tree_calibration.columnCount()):
            self._widget.tree_calibration.resizeColumnToContents(col)

    def btn_save_clicked_(self):
        path_to_config = "~"
        try:
            path_to_config = roslib.packages.get_pkg_dir("sr_robot_lib") + "/config"
        except:
            rospy.logwarn("couldnt find the sr_edc_controller_configuration package")

        filter_files = "*.yaml"
        filename, _ = QFileDialog.getOpenFileName(self._widget.tree_calibration, self._widget.tr('Save Controller Settings'),
                                                  self._widget.tr(path_to_config),
                                                  self._widget.tr(filter_files))

        if filename == "":
            return

        self.hand_model.save( filename )

    def btn_load_clicked_(self):
        path_to_config = "~"
        try:
            path_to_config = roslib.packages.get_pkg_dir("sr_robot_lib") + "/config"
        except:
            rospy.logwarn("couldnt find the sr_edc_controller_configuration package")

        filter_files = "*.yaml"
        filename, _ = QFileDialog.getOpenFileName(self._widget.tree_calibration, self._widget.tr('Save Controller Settings'),
                                                  self._widget.tr(path_to_config),
                                                  self._widget.tr(filter_files))

        if filename == "":
            return

        self.hand_model.load( filename )

    def btn_joint_0s_clicked_(self):
        self.hand_model.calibrate_joint0s( self._widget.btn_joint_0s )

    def _unregisterPublisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

        self.hand_model.unregister()

    def eventFilter(self, obj, event):
        if obj is self._widget and event.type() == QEvent.Close:
            # TODO: ignore() should not be necessary when returning True
            event.ignore()
            self.deleteLater()
            return True
        return QObject.eventFilter(self, obj, event)

    def close_plugin(self):
        self._unregisterPublisher()
        self._widget.close()
        self._widget.deleteLater()

    def save_settings(self, global_settings, perspective_settings):
        print "saving settings"
        #topic = self._widget.topic_line_edit.text()
        #perspective_settings.set_value('topic', topic)

    def restore_settings(self, global_settings, perspective_settings):
        print "restoring settings"
        #topic = perspective_settings.value('topic', '/cmd_vel')


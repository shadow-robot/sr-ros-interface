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
from QtGui import QDockWidget, QShortcut, QTreeWidgetItem, QCheckBox, QSpinBox, QDoubleSpinBox, QFileDialog, QMessageBox

from QtCore import QVariant

import roslib
roslib.load_manifest('sr_gui_controller_tuner')
import rospy

from sr_controller_tuner import SrControllerTunerLib

class SrGuiControllerTuner(QObject):

    def __init__(self, parent, plugin_context):
        super(SrGuiControllerTuner, self).__init__(parent)
        self.setObjectName('SrGuiControllerTuner')

        self.controller_type = None
        self._publisher = None
        main_window = plugin_context.main_window()
        self._widget = QDockWidget(main_window)

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../uis/SrGuiControllerTuner.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrControllerTunerUi')
        if plugin_context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % plugin_context.serial_number()))
        main_window.addDockWidget(Qt.RightDockWidgetArea, self._widget)

        # trigger deleteLater for plugin when _widget is closed
        self._widget.installEventFilter(self)

        #stores the controllers in the same order as the dropdown
        self.controllers_in_dropdown = []

        #stores a dictionary of the widgets containing the data for
        # the different controllers.
        self.ctrl_widgets = {}

        #a library which helps us doing the real work.
        self.sr_controller_tuner_lib_ = SrControllerTunerLib( os.path.join(os.path.dirname(os.path.realpath(__file__)), '../data/controller_settings.xml') )

        #refresh the controllers once
        self.on_btn_refresh_ctrl_clicked_()
        #attach the button pressed to its action
        self._widget.btn_refresh_ctrl.pressed.connect(self.on_btn_refresh_ctrl_clicked_)
        self._widget.dropdown_ctrl.activated.connect(self.on_changed_controller_type_)

        self._widget.btn_save_selected.pressed.connect(self.on_btn_save_selected_clicked_)
        self._widget.btn_save_all.pressed.connect(self.on_btn_save_all_clicked_)
        self._widget.btn_select_file_path.pressed.connect(self.on_btn_select_file_path_clicked_)

        self._widget.btn_set_selected.pressed.connect(self.on_btn_set_selected_clicked_)
        self._widget.btn_set_all.pressed.connect(self.on_btn_set_all_clicked_)

    @Slot(str)
    def on_changed_controller_type_(self, index):
        self.refresh_controller_tree_( self.controllers_in_dropdown[index] )

    def on_btn_select_file_path_clicked_(self):
        path_to_config = "~"
        try:
            path_to_config = roslib.packages.get_pkg_dir("sr_edc_controller_configuration")
        except:
            rospy.logwarn("couldnt find the sr_edc_controller_configuration package")

        filter_files = "*.yaml"
        filename = path_to_config + "/motor_effort_controllers.yaml"
                   #QFileDialog.getOpenFileName( self._widget.tr("Save Controller Settings"),
                   #                             self._widget.tr(path_to_config),
                   #                             self._widget.tr(filter_files) )
        if filename == "":
            return

        self._widget.txt_file_path.setText(filename)

        #enable the save buttons once a file has
        # been chosen
        self._widget.btn_save_all.setEnabled(True)
        self._widget.btn_save_selected.setEnabled(True)


    def on_btn_save_selected_clicked_(self):
        selected_items = self._widget.tree_ctrl_settings.selectedItems()

        if len( selected_items ) == 0:
            QMessageBox.warning(self._widget.tree_ctrl_settings, "Warning", "No motors selected.")

        for it in selected_items:
            if str(it.text(1)) != "":
                self.save_controller( str(it.text(1)) )

    def on_btn_save_all_clicked_(self):
        for motor in self.ctrl_widgets.keys():
            self.save_controller( motor )

    def on_btn_set_selected_clicked_(self):
        selected_items = self._widget.tree_ctrl_settings.selectedItems()

        if len( selected_items ) == 0:
            QMessageBox.warning(self._widget.tree_ctrl_settings, "Warning", "No motors selected.")

        for it in selected_items:
            if str(it.text(1)) != "":
                self.set_controller( str(it.text(1)) )

    def on_btn_set_all_clicked_(self):
        for motor in self.ctrl_widgets.keys():
            self.set_controller( motor )

    def on_btn_refresh_ctrl_clicked_(self):
        ctrls = self.sr_controller_tuner_lib_.get_ctrls()
        self.controllers_in_dropdown = []
        self._widget.dropdown_ctrl.clear()
        for ctrl in ctrls:
            self._widget.dropdown_ctrl.addItem(ctrl)
            self.controllers_in_dropdown.append(ctrl)

        self.refresh_controller_tree_()

    def set_controller(self, joint_name):
        """
        Sets the current values for the given controller
        using the ros service.
        """
        dict_of_widgets = self.ctrl_widgets[joint_name]

        settings = {}
        for item in dict_of_widgets.items():
            if item[0] == "sign":
                if item[1].checkState() == Qt.Checked:
                    settings["sign"] = 1
                else:
                    settings["sign"] = 0
            else:
                settings[item[0]] = item[1].value()

        #uses the library to call the service properly
        success = self.sr_controller_tuner_lib_.set_controller(joint_name, self.controller_type, settings)
        if success == False:
            QMessageBox.warning(self._widget.tree_ctrl_settings, "Warning", "Failed to set the PID values for joint "+ joint_name +".")


    def save_controller(self, joint_name):
        """
        Sets the current values for the given controller
        using the ros service.
        """
        dict_of_widgets = self.ctrl_widgets[joint_name]

        settings = {}
        for item in dict_of_widgets.items():
            if item[0] == "sign":
                if item[1].checkState() == Qt.Checked:
                    settings["sign"] = 1
                else:
                    settings["sign"] = 0
            else:
                settings[item[0]] = item[1].value()

        #uses the library to call the service properly
        self.sr_controller_tuner_lib_.save_controller(joint_name, self.controller_type, settings)


    def refresh_controller_tree_(self, controller_type = "Motor Force"):
        """
        Get the controller settings and their ranges and display them in
        the tree.
        """
        self.controller_type = controller_type
        ctrl_settings = self.sr_controller_tuner_lib_.get_controller_settings( controller_type )

        self._widget.tree_ctrl_settings.clear()
        self._widget.tree_ctrl_settings.setColumnCount(ctrl_settings.nb_columns)

        tmp_headers = []
        for header in ctrl_settings.headers:
            tmp_headers.append( header["name"] )
        self._widget.tree_ctrl_settings.setHeaderLabels( tmp_headers )

        hand_item = QTreeWidgetItem(ctrl_settings.hand_item)
        self._widget.tree_ctrl_settings.addTopLevelItem(hand_item)
        for index_finger,finger_settings in enumerate(ctrl_settings.fingers):
            finger_item = QTreeWidgetItem( hand_item, finger_settings )
            self._widget.tree_ctrl_settings.addTopLevelItem(finger_item)
            for motor_settings in ctrl_settings.motors[index_finger]:
                motor_name = motor_settings[1]

                motor_item = QTreeWidgetItem( finger_item, motor_settings )
                self._widget.tree_ctrl_settings.addTopLevelItem(motor_item)

                parameter_values = self.sr_controller_tuner_lib_.load_parameters( controller_type, motor_name )
                self.ctrl_widgets[ motor_name ] = {}

                for index_header,header in enumerate(ctrl_settings.headers):
                    if header["type"] == "Bool":
                        check_box = QCheckBox()

                        if parameter_values["sign"] == 1.0:
                            check_box.setChecked(True)

                        check_box.setToolTip("Check if you want a negative sign\n(if the motor is being driven\n the wrong way around).")
                        self._widget.tree_ctrl_settings.setItemWidget(  motor_item, index_header, check_box )

                        self.ctrl_widgets[ motor_name ]["sign"] = check_box


                    if header["type"] == "Int":
                        spin_box = QSpinBox()
                        spin_box.setRange(int( header["min"] ), int( header["max"] ))

                        param_name = header["name"].lower()
                        spin_box.setValue( int(parameter_values[ param_name ] ) )
                        self.ctrl_widgets[ motor_name ][param_name] = spin_box

                        self._widget.tree_ctrl_settings.setItemWidget(  motor_item, index_header, spin_box )

                    if header["type"] == "Float":
                        spin_box = QDoubleSpinBox()
                        spin_box.setRange( -65535.0, 65535.0 )
                        spin_box.setDecimals(3)

                        param_name = header["name"].lower()
                        spin_box.setValue(float(parameter_values[param_name]))
                        self.ctrl_widgets[ motor_name ][param_name] = spin_box

                        self._widget.tree_ctrl_settings.setItemWidget(  motor_item, index_header, spin_box )

                motor_item.setExpanded(True)
            finger_item.setExpanded(True)
        hand_item.setExpanded(True)




    #########
    #Default methods for the rosgui plugins

    def _unregisterPublisher(self):
        if self._publisher is not None:
            self._publisher.unregister()
            self._publisher = None

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


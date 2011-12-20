# Copyright (c) 2011, Dirk Thomas, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division
import os

import roslib
roslib.load_manifest('sr_gui_change_controllers')
import rospy

from rosgui.QtBindingHelper import loadUi
from QtCore import QEvent, QObject, Qt, QTimer, Slot
from QtGui import QDockWidget, QShortcut, QMessageBox
from pr2_mechanism_msgs.srv import ListControllers, SwitchController, LoadController

class SrGuiChangeControllers(QObject):

    controllers = {"effort": ["sh_ffj0_effort_controller", "sh_ffj3_effort_controller", "sh_ffj4_effort_controller", "sh_mfj0_effort_controller", "sh_mfj3_effort_controller", "sh_mfj4_effort_controller", "sh_rfj0_effort_controller", "sh_rfj3_effort_controller", "sh_rfj4_effort_controller", "sh_lfj0_effort_controller", "sh_lfj3_effort_controller", "sh_lfj4_effort_controller", "sh_lfj5_effort_controller", "sh_thj1_effort_controller", "sh_thj2_effort_controller", "sh_thj3_effort_controller", "sh_thj4_effort_controller", "sh_thj5_effort_controller", "sh_wrj1_effort_controller", "sh_wrj2_effort_controller"],
                   "position": ["sh_ffj0_position_controller", "sh_ffj3_position_controller", "sh_ffj4_position_controller", "sh_mfj0_position_controller", "sh_mfj3_position_controller", "sh_mfj4_position_controller", "sh_rfj0_position_controller", "sh_rfj3_position_controller", "sh_rfj4_position_controller", "sh_lfj0_position_controller", "sh_lfj3_position_controller", "sh_lfj4_position_controller", "sh_lfj5_position_controller", "sh_thj1_position_controller", "sh_thj2_position_controller", "sh_thj3_position_controller", "sh_thj4_position_controller", "sh_thj5_position_controller", "sh_wrj1_position_controller", "sh_wrj2_position_controller"],
                   "mixed": ["sh_ffj0_mixed_position_velocity_controller", "sh_ffj3_mixed_position_velocity_controller", "sh_ffj4_mixed_position_velocity_controller", "sh_mfj0_mixed_position_velocity_controller", "sh_mfj3_mixed_position_velocity_controller", "sh_mfj4_mixed_position_velocity_controller", "sh_rfj0_mixed_position_velocity_controller", "sh_rfj3_mixed_position_velocity_controller", "sh_rfj4_mixed_position_velocity_controller", "sh_lfj0_mixed_position_velocity_controller", "sh_lfj3_mixed_position_velocity_controller", "sh_lfj4_mixed_position_velocity_controller", "sh_lfj5_mixed_position_velocity_controller", "sh_thj1_mixed_position_velocity_controller", "sh_thj2_mixed_position_velocity_controller", "sh_thj3_mixed_position_velocity_controller", "sh_thj4_mixed_position_velocity_controller", "sh_thj5_mixed_position_velocity_controller", "sh_wrj1_mixed_position_velocity_controller", "sh_wrj2_mixed_position_velocity_controller"],
                   "velocity": ["sh_ffj0_velocity_controller", "sh_ffj3_velocity_controller", "sh_ffj4_velocity_controller", "sh_mfj0_velocity_controller", "sh_mfj3_velocity_controller", "sh_mfj4_velocity_controller", "sh_rfj0_velocity_controller", "sh_rfj3_velocity_controller", "sh_rfj4_velocity_controller", "sh_lfj0_velocity_controller", "sh_lfj3_velocity_controller", "sh_lfj4_velocity_controller", "sh_lfj5_velocity_controller", "sh_thj1_velocity_controller", "sh_thj2_velocity_controller", "sh_thj3_velocity_controller", "sh_thj4_velocity_controller", "sh_thj5_velocity_controller", "sh_wrj1_velocity_controller", "sh_wrj2_velocity_controller"],
                   "stop": []}

    def __init__(self, parent, plugin_context):
        super(SrGuiChangeControllers, self).__init__(parent)
        self.setObjectName('SrGuiChangeControllers')

        self._publisher = None
        main_window = plugin_context.main_window()
        self._widget = QDockWidget(main_window)

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../uis/SrChangeControllers.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrChangeControllersUi')
        if plugin_context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % plugin_context.serial_number()))
        main_window.addDockWidget(Qt.RightDockWidgetArea, self._widget)

        # trigger deleteLater for plugin when _widget is closed
        self._widget.installEventFilter(self)

        #attaching the button press event to their actions
        self._widget.btn_stop.pressed.connect(self.on_stop_ctrl_clicked_)
        self._widget.btn_effort.pressed.connect(self.on_effort_ctrl_clicked_)
        self._widget.btn_position.pressed.connect(self.on_position_ctrl_clicked_)
        self._widget.btn_mixed.pressed.connect(self.on_mixed_ctrl_clicked_)
        self._widget.btn_velocity.pressed.connect(self.on_velocity_ctrl_clicked_)

    @Slot(str)
    def on_stop_ctrl_clicked_(self):
        self.change_ctrl( "stopped" )

    def on_effort_ctrl_clicked_(self):
        self.change_ctrl( "effort" )

    def on_position_ctrl_clicked_(self):
        self.change_ctrl( "position" )

    def on_mixed_ctrl_clicked_(self):
        self.change_ctrl( "mixed" )

    def on_velocity_ctrl_clicked_(self):
        self.change_ctrl( "velocity" )

    def change_ctrl(self, controller):
        success = True
        list_controllers = rospy.ServiceProxy('pr2_controller_manager/list_controllers', ListControllers)
        try:
            resp1 = list_controllers()
        except rospy.ServiceException:
            success = False

        if success:
            current_controllers = []
            all_loaded_controllers = resp1.controllers
            for state,tmp_contrl in zip(resp1.state,resp1.controllers):
                if state == "running":
                    current_controllers.append(tmp_contrl)

            controllers_to_start = self.controllers[controller]

            load_controllers = rospy.ServiceProxy('/pr2_controller_manager/load_controller', LoadController)
            for load_control in controllers_to_start:
                if load_control not in all_loaded_controllers:
                    try:
                        resp1 = load_controllers(load_control)
                    except rospy.ServiceException:
                        success = False
                    if not resp1.ok:
                        success = False

            switch_controllers = rospy.ServiceProxy('pr2_controller_manager/switch_controller', SwitchController)
            try:
                resp1 = switch_controllers(controllers_to_start, current_controllers, 2)
            except rospy.ServiceException:
                success = False

            if not resp1.ok:
                success = False

        if not success:
            QMessageBox.warning(self._widget, "Warning", "Failed to change the controllers.")

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


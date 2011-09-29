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

from PyQt4 import QtCore, QtGui, Qt

from config import Config
from generic_plugin import GenericPlugin
from functools import partial
from pr2_mechanism_msgs.srv import ListControllers, SwitchController, LoadController

class ChangeControllers(GenericPlugin):
    """
    Swaps between the different controllers.
    """
    name = "Change Controllers"

    def __init__(self):
        GenericPlugin.__init__(self)

        self.controllers = {"effort": ["sh_ffj0_effort_controller", "sh_ffj3_effort_controller", "sh_ffj4_effort_controller", "sh_mfj0_effort_controller", "sh_mfj3_effort_controller", "sh_mfj4_effort_controller", "sh_rfj0_effort_controller", "sh_rfj3_effort_controller", "sh_rfj4_effort_controller", "sh_lfj0_effort_controller", "sh_lfj3_effort_controller", "sh_lfj4_effort_controller", "sh_lfj5_effort_controller", "sh_thj1_effort_controller", "sh_thj2_effort_controller", "sh_thj3_effort_controller", "sh_thj4_effort_controller", "sh_thj5_effort_controller", "sh_wrj1_effort_controller", "sh_wrj2_effort_controller"],
                            "position": ["sh_ffj0_position_controller", "sh_ffj3_position_controller", "sh_ffj4_position_controller", "sh_mfj0_position_controller", "sh_mfj3_position_controller", "sh_mfj4_position_controller", "sh_rfj0_position_controller", "sh_rfj3_position_controller", "sh_rfj4_position_controller", "sh_lfj0_position_controller", "sh_lfj3_position_controller", "sh_lfj4_position_controller", "sh_lfj5_position_controller", "sh_thj1_position_controller", "sh_thj2_position_controller", "sh_thj3_position_controller", "sh_thj4_position_controller", "sh_thj5_position_controller", "sh_wrj1_position_controller", "sh_wrj2_position_controller"],
                            "mixed position/velocity": ["sh_ffj0_mixed_position_velocity_controller", "sh_ffj3_mixed_position_velocity_controller", "sh_ffj4_mixed_position_velocity_controller", "sh_mfj0_mixed_position_velocity_controller", "sh_mfj3_mixed_position_velocity_controller", "sh_mfj4_mixed_position_velocity_controller", "sh_rfj0_mixed_position_velocity_controller", "sh_rfj3_mixed_position_velocity_controller", "sh_rfj4_mixed_position_velocity_controller", "sh_lfj0_mixed_position_velocity_controller", "sh_lfj3_mixed_position_velocity_controller", "sh_lfj4_mixed_position_velocity_controller", "sh_lfj5_mixed_position_velocity_controller", "sh_thj1_mixed_position_velocity_controller", "sh_thj2_mixed_position_velocity_controller", "sh_thj3_mixed_position_velocity_controller", "sh_thj4_mixed_position_velocity_controller", "sh_thj5_mixed_position_velocity_controller", "sh_wrj1_mixed_position_velocity_controller", "sh_wrj2_mixed_position_velocity_controller"],
                            "velocity": ["sh_ffj0_velocity_controller", "sh_ffj3_velocity_controller", "sh_ffj4_velocity_controller", "sh_mfj0_velocity_controller", "sh_mfj3_velocity_controller", "sh_mfj4_velocity_controller", "sh_rfj0_velocity_controller", "sh_rfj3_velocity_controller", "sh_rfj4_velocity_controller", "sh_lfj0_velocity_controller", "sh_lfj3_velocity_controller", "sh_lfj4_velocity_controller", "sh_lfj5_velocity_controller", "sh_thj1_velocity_controller", "sh_thj2_velocity_controller", "sh_thj3_velocity_controller", "sh_thj4_velocity_controller", "sh_thj5_velocity_controller", "sh_wrj1_velocity_controller", "sh_wrj2_velocity_controller"],
                            "stop": []}



        self.layout = QtGui.QHBoxLayout()
        self.frame = QtGui.QFrame()
        self.buttons = {}
        for controller in self.controllers.keys():
            tmp_btn = QtGui.QPushButton(controller)
            self.buttons[controller] = tmp_btn
            tmp_btn.clicked.connect(partial(self.clicked, controller))
            self.layout.addWidget( tmp_btn )
        self.frame.setLayout( self.layout )
        self.window.setWidget( self.frame )

        self.dependencies = None

    def clicked(self, controller):
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
            QtGui.QMessageBox.warning(self.frame, "Warning", "Failed to change the controllers.")

    def activate(self):
        GenericPlugin.activate(self)

    def on_close(self):
        GenericPlugin.on_close(self)

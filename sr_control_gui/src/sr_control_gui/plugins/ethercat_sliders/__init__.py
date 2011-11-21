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

from joint_slider import Joint, LightJointSlider
from config import Config
import math

from PyQt4 import QtCore, QtGui, Qt
from std_msgs.msg import Float64
from pr2_mechanism_msgs.srv import ListControllers


class EtherCATJointSlider(LightJointSlider):
    """
    Sliders to send a demand to a loaded controller of the etherCAT
    hand.
    """
    name = "EtherCAT Sliders "

    def __init__(self):
        LightJointSlider.__init__(self)

        self.refresh_btn = QtGui.QPushButton()
        self.refresh_btn.setText("Refresh Controllers")
        self.refresh_btn.setToolTip("Check with the controller manager to see which controller is currently running")
        self.control_frame.connect(self.refresh_btn, QtCore.SIGNAL('clicked()'),self.refresh_controllers)
        self.control_layout.addWidget(self.refresh_btn)

    def activate(self):
        self.refresh_controllers()

    def refresh_controllers(self):
        rospy.wait_for_service('/pr2_controller_manager/list_controllers')
        controllers = rospy.ServiceProxy('/pr2_controller_manager/list_controllers', ListControllers)
        resp = None
        try:
            resp = controllers()
        except rospy.ServiceException, e:
            print "Service did not process request: %s"%str(e)

        self.controller_type = "default"
        controllers_tmp = []
        if resp != None:
            for state,controller in zip(resp.state,resp.controllers):
                if state == "running":
                    split = controller.split("_")
                    joint_name = split[1]
                    controllers_tmp.append( [joint_name, controller + "/command"] )
                    self.controller_type = split[2]

        controllers_tmp.sort()
        self.name = "EtherCAT Sliders "
        self.name += self.controller_type

        joints_list = []
        self.publishers = {}

        control_range = {"default":[-100, 100],
                         "effort":[-500, 500],
                         "position":[-180, 180],
                         "mixed":[-180, 180]}

        min_ = control_range["default"][0]
        max_ = control_range["default"][1]
        if control_range.has_key(self.controller_type):
            min_ = control_range[self.controller_type][0]
            max_ = control_range[self.controller_type][1]

        for slider in self.sliders:
            slider.timer.stop()
            slider.setParent(None)
            del slider
        self.sliders = []
        if self.super_slider != None:
            self.super_slider.setParent(None)
            del self.super_slider
            self.super_slider = None

        joints_list = []
        for j in controllers_tmp:
            joints_list.append( Joint(j[0], min_, max_) )
            self.publishers[j[0]] = rospy.Publisher(j[1], Float64)

        LightJointSlider.activate(self, self.name, joints_list)


    def on_close(self):
        LightJointSlider.on_close(self)
        self.publishers = {}
        self.name = "EtherCAT Sliders "


    def sendupdate(self, dict):
        if( self.controller_type == "position" ):
            for item in dict.items():
                self.publishers[item[0]].publish(Float64( math.radians( item[1] ) ))
        else:
            for item in dict.items():
                self.publishers[item[0]].publish(Float64( item[1] ))



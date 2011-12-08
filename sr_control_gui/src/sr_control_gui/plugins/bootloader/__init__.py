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

import roslib.packages
import time

from generic_plugin import GenericPlugin
from sr_robot_msgs.srv import SimpleMotorFlasher, SimpleMotorFlasherResponse
from diagnostic_msgs.msg import DiagnosticArray

from PyQt4 import QtCore, QtGui, Qt

class MotorFlasher(QtCore.QThread):
    def __init__(self, parent, nb_motors_to_program):
        QtCore.QThread.__init__(self, None)
        self.parent = parent
        self.nb_motors_to_program = nb_motors_to_program

        rospy.wait_for_service('SimpleMotorFlasher')
        self.flasher_service = rospy.ServiceProxy('SimpleMotorFlasher', SimpleMotorFlasher)

    def run(self):
        programmed_motors = 0.
        for motor in self.parent.motors:
            if motor.checkbox.checkState() == QtCore.Qt.Checked:
                resp = SimpleMotorFlasherResponse.FAIL
                try:
                    resp = self.flasher_service( str(self.parent.firmware_path), motor.motor_index )

                except rospy.ServiceException, e:
                    self.emit( QtCore.SIGNAL("failed(QString)"),
                               QtCore.QString( "Service did not process request: %s"%str(e) ) )
                    return

                if resp == SimpleMotorFlasherResponse.FAIL:
                    self.emit( QtCore.SIGNAL("failed(QString)"),
                               QtCore.QString( "Failed to program the motor" ) )
                    return
                programmed_motors += 1.

                self.emit( QtCore.SIGNAL("motor_finished(QPoint)"),
                               QtCore.QPoint( programmed_motors / self.nb_motors_to_program * 100. , 0.0 ) )


class Motor(QtGui.QFrame):
    def __init__(self, parent, motor_name, motor_index):
        QtGui.QFrame.__init__(self, parent)

        self.motor_name = motor_name
        self.motor_index = motor_index

        self.layout = QtGui.QHBoxLayout()

        self.checkbox = QtGui.QCheckBox("", self)
        self.layout.addWidget(self.checkbox)

        self.label = QtGui.QLabel( motor_name + " [" + str(motor_index) +"]" )
        self.label.setToolTip("Motor name and motor index")
        self.layout.addWidget(self.label)

        self.revision_label = QtGui.QLabel( "" )
        self.revision_label.setToolTip("Svn Revision")
        self.layout.addWidget(self.revision_label)

        self.setLayout(self.layout)

class Bootloader(GenericPlugin):
    """
    Bootload the given firmware into the selected motors.
    """
    name = "Bootloader"

    def __init__(self):
        GenericPlugin.__init__(self)

        self.diag_sub = None
        self.firmware_path = None

        self.frame = QtGui.QFrame()
        self.layout = QtGui.QVBoxLayout()

        self.file_frame = QtGui.QFrame()
        self.file_layout = QtGui.QHBoxLayout()

        self.file_btn = QtGui.QPushButton()
        self.file_btn.setText("Choose Firmware")
        self.file_btn.setToolTip("Choose which compiled firmware you want to load.")
        self.file_frame.connect(self.file_btn, QtCore.SIGNAL('clicked()'),self.choose_firmware)
        self.file_layout.addWidget(self.file_btn)

        self.file_label = QtGui.QLabel("No firmware chosen")
        self.file_layout.addWidget(self.file_label)

        self.file_frame.setLayout(self.file_layout)
        self.layout.addWidget(self.file_frame)

        self.motors_frame = QtGui.QFrame()
        self.motors_layout = QtGui.QGridLayout()

        self.motors = []
        self.motors_frame.setLayout(self.motors_layout)
        self.layout.addWidget(self.motors_frame)

        self.frame_select_and_server_rev = QtGui.QFrame()
        self.layout_select_and_server_rev = QtGui.QHBoxLayout()

        self.all_selected = False
        self.select_all_btn =  QtGui.QPushButton()
        self.select_all_btn.setText("Select/Deselect All")
        self.select_all_btn.setToolTip("Select or deselect all motors.")
        self.file_frame.connect(self.select_all_btn, QtCore.SIGNAL('clicked()'),self.select_all)
        self.layout_select_and_server_rev.addWidget(self.select_all_btn)

        self.server_revision_label = QtGui.QLabel("")
        self.layout_select_and_server_rev.addWidget(self.server_revision_label)

        self.frame_select_and_server_rev.setLayout(self.layout_select_and_server_rev)
        self.layout.addWidget(self.frame_select_and_server_rev)

        self.program_frame = QtGui.QFrame()
        self.program_layout = QtGui.QHBoxLayout()

        self.program_btn = QtGui.QPushButton()
        self.program_btn.setText("Program Motors")
        self.program_btn.setToolTip("Program the selected motors with the choosen firmware")
        self.program_btn.setEnabled(False)

        self.program_frame.connect(self.program_btn, QtCore.SIGNAL('clicked()'),self.program_motors)
        self.program_layout.addWidget(self.program_btn)

        self.progress_bar = QtGui.QProgressBar()
        self.program_layout.addWidget(self.progress_bar)

        self.program_frame.setLayout(self.program_layout)
        self.layout.addWidget(self.program_frame)

        self.frame.setLayout(self.layout)
        self.window.setWidget(self.frame)

    def choose_firmware(self):
        path_to_released_firmware = ""
        try:
            path_to_released_firmware = roslib.packages.get_pkg_dir("sr_external_dependencies")
        except:
            rospy.logwarn("couldnt find the sr_external_dependencies package")
            pass
        filename = QtGui.QFileDialog.getOpenFileName(self.file_frame, Qt.QString("Firmware file"), Qt.QString(path_to_released_firmware + "/compiled_firmware/released_firmware"), Qt.QString("*.hex") )
        if filename == "":
            if self.firmware_path == None:
                self.program_btn.setEnabled(False)
            return
        self.firmware_path = filename
        self.file_label.setText(filename.split("/")[-1])
        self.program_btn.setEnabled(True)

    def select_all(self):
        if self.all_selected:
            for motor in self.motors:
                motor.checkbox.setCheckState( QtCore.Qt.Unchecked )
                self.all_selected = False
        else:
            for motor in self.motors:
                motor.checkbox.setCheckState( QtCore.Qt.Checked )
                self.all_selected = True

    def program_motors(self):
        self.progress_bar.setValue(0)
        nb_motors_to_program = 0.
        for motor in self.motors:
            if motor.checkbox.checkState() == QtCore.Qt.Checked:
                nb_motors_to_program += 1.
        if nb_motors_to_program == 0.:
            QtGui.QMessageBox.warning(self.frame, "Warning", "No motors selected for flashing.")
            return

        self.cursor = Qt.QCursor()
        self.cursor.setShape(QtCore.Qt.WaitCursor)
        self.window.setCursor(self.cursor)

        self.motor_flasher = MotorFlasher(self, nb_motors_to_program)
        self.frame.connect(self.motor_flasher, QtCore.SIGNAL("finished()"), self.finished_programming_motors)
        self.frame.connect(self.motor_flasher, QtCore.SIGNAL("motor_finished(QPoint)"), self.one_motor_finished)
        self.frame.connect(self.motor_flasher, QtCore.SIGNAL("failed(QString)"), self.failed_programming_motors)

        for motor in self.motors:
            motor.checkbox.setEnabled(False)
        self.file_btn.setEnabled(False)
        self.select_all_btn.setEnabled(False)
        self.program_btn.setEnabled(False)

        self.motor_flasher.start()

    def one_motor_finished(self, point):
        self.progress_bar.setValue( int(point.x()) )


    def finished_programming_motors(self):
        for motor in self.motors:
            motor.checkbox.setEnabled(True)
        self.file_btn.setEnabled(True)
        self.select_all_btn.setEnabled(True)
        self.program_btn.setEnabled(True)

        self.cursor.setShape(QtCore.Qt.ArrowCursor)
        self.window.setCursor(self.cursor)

    def failed_programming_motors(self, message):
        QtGui.QMessageBox.warning(self.frame, "Warning", message)

    def populate_motors(self):
        if rospy.has_param("joint_to_motor_mapping"):
            joint_to_motor_mapping = rospy.get_param("joint_to_motor_mapping")

        joint_names = [ ["FFJ0", "FFJ1", "FFJ2", "FFJ3", "FFJ4"],
                        ["MFJ0", "MFJ1", "MFJ2", "MFJ3", "MFJ4"],
                        ["RFJ0", "RFJ1", "RFJ2", "RFJ3", "RFJ4"],
                        ["LFJ0", "LFJ1", "LFJ2", "LFJ3", "LFJ4", "LFJ5"],
                        ["THJ1", "THJ2", "THJ3", "THJ4"],
                        ["THJ5"],
                        ["WRJ1", "WRJ2"] ]

        row = 0
        col = 0
        index_jtm_mapping = 0
        for finger in joint_names:
            col = 0
            for joint_name in finger:
                motor_index = joint_to_motor_mapping[index_jtm_mapping]
                if motor_index != -1:
                    motor = Motor(self.motors_frame, joint_name, motor_index)
                    self.motors_layout.addWidget(motor, row, col)
                    self.motors.append( motor )
                    col += 1
                index_jtm_mapping += 1
            row += 1

    def activate(self):
        GenericPlugin.activate(self)
        self.set_icon(self.parent.parent.rootPath + '/images/icons/iconHand.png')
        self.populate_motors()

        self.server_revision = 0
        self.diag_sub = rospy.Subscriber("/diagnostics", DiagnosticArray, self.diagnostics_callback)
        Qt.QTimer.singleShot(0, self.window.adjustSize)
        Qt.QTimer.singleShot(1500, self.window.adjustSize)

    def diagnostics_callback(self, msg):
        for status in msg.status:
            for motor in self.motors:
                if motor.motor_name in status.name:
                    for key_values in status.values:
                        if "Firmware svn revision" in key_values.key:
                            server_current_modified = key_values.value.split(" / ")

                            if server_current_modified[0] > self.server_revision:
                                self.server_revision = int( server_current_modified[0].strip() )

                            palette = motor.revision_label.palette();
                            palette.setColor(motor.revision_label.foregroundRole(), QtCore.Qt.green)
                            if server_current_modified[0].strip() != server_current_modified[1].strip():
                                palette.setColor(motor.revision_label.foregroundRole(), QtGui.QColor(255, 170, 23) )
                                motor.revision_label.setPalette(palette);

                            if "True" in server_current_modified[2]:
                                palette.setColor(motor.revision_label.foregroundRole(), QtCore.Qt.red)
                                motor.revision_label.setText( "svn: "+ server_current_modified[1] + " [M]" )
                                motor.revision_label.setPalette(palette);
                            else:
                                motor.revision_label.setText( " svn: " + server_current_modified[1] )
                                motor.revision_label.setPalette(palette);

        self.server_revision_label.setText( "  Server svn revision: " +  str(self.server_revision) )

    def on_close(self):
        if self.diag_sub != None:
            self.diag_sub.unregister()
            self.diag_sub = None

        for motor in self.motors:
            motor.setParent(None)
        self.motors = []

        GenericPlugin.on_close(self)

























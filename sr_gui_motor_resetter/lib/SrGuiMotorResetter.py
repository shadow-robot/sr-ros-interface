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

from __future__ import division
import os

import roslib
roslib.load_manifest('sr_gui_motor_resetter')
import rospy
from std_srvs.srv import Empty
from diagnostic_msgs.msg import DiagnosticArray

from rosgui.QtBindingHelper import loadUi
from QtCore import QEvent, QObject, Qt, QTimer, Slot, QThread, SIGNAL, QPoint
from QtGui import QDockWidget, QShortcut, QMessageBox, QFrame, QHBoxLayout, QCheckBox, QLabel, QCursor, QColor

class MotorFlasher(QThread):
    def __init__(self, parent, nb_motors_to_program):
        QThread.__init__(self, None)
        self.parent = parent
        self.nb_motors_to_program = nb_motors_to_program

    def run(self):
        programmed_motors = 0
        for motor in self.parent.motors:
            if motor.checkbox.checkState() == Qt.Checked:
                try:
                    print("resetting: /realtime_loop/reset_motor_"+motor.motor_name)
                    self.flasher_service = rospy.ServiceProxy('/realtime_loop/reset_motor_'+motor.motor_name, Empty)
                    resp = self.flasher_service()
                except rospy.ServiceException, e:
                    self.emit( SIGNAL("failed(QString)"),
                               QString( "Service did not process request: %s"%str(e) ) )
                    return

                programmed_motors += 1
                self.emit( SIGNAL("motor_finished(QPoint)"), QPoint( programmed_motors, 0.0 ) )

class Motor(QFrame):
    def __init__(self, parent, motor_name, motor_index):
        QFrame.__init__(self, parent)

        self.motor_name = motor_name
        self.motor_index = motor_index

        self.layout = QHBoxLayout()

        self.checkbox = QCheckBox("", self)
        self.layout.addWidget(self.checkbox)

        self.label = QLabel( motor_name + " [" + str(motor_index) +"]" )
        self.label.setToolTip("Motor name and motor index")
        self.layout.addWidget(self.label)

        self.revision_label = QLabel( "" )
        self.revision_label.setToolTip("Svn Revision")
        self.layout.addWidget(self.revision_label)

        self.setLayout(self.layout)


class SrGuiMotorResetter(QObject):

    def __init__(self, parent, plugin_context):
        super(SrGuiMotorResetter, self).__init__(parent)
        self.setObjectName('SrGuiMotorResetter')

        self._publisher = None
        main_window = plugin_context.main_window()
        self._widget = QDockWidget(main_window)

        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../uis/SrGuiMotorResetter.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('SrMotorResetterUi')
        if plugin_context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % plugin_context.serial_number()))
        main_window.addDockWidget(Qt.RightDockWidgetArea, self._widget)

        # trigger deleteLater for plugin when _widget is closed
        self._widget.installEventFilter(self)

        # motors_frame is defined in the ui file with a grid layout
        self.motors = []
        self.motors_frame = self._widget.motors_frame
        self.populate_motors()
        self.progress_bar = self._widget.motors_progress_bar

        self.server_revision = 0
        self.diag_sub = rospy.Subscriber("/diagnostics", DiagnosticArray, self.diagnostics_callback)

        # Bind button clicks
        self._widget.btn_select_all.pressed.connect(self.on_select_all_pressed)
        self._widget.btn_select_none.pressed.connect(self.on_select_none_pressed)
        self._widget.btn_reset_motors.pressed.connect(self.on_reset_motors_pressed)


    def populate_motors(self):
        if rospy.has_param("joint_to_motor_mapping"):
            joint_to_motor_mapping = rospy.get_param("joint_to_motor_mapping")

        joint_names = [
                ["FFJ0", "FFJ1", "FFJ2", "FFJ3", "FFJ4"],
                ["MFJ0", "MFJ1", "MFJ2", "MFJ3", "MFJ4"],
                ["RFJ0", "RFJ1", "RFJ2", "RFJ3", "RFJ4"],
                ["LFJ0", "LFJ1", "LFJ2", "LFJ3", "LFJ4", "LFJ5"],
                ["THJ1", "THJ2", "THJ3", "THJ4","THJ5"],
                ["WRJ1", "WRJ2"]
            ]

        row = 0
        col = 0
        index_jtm_mapping = 0
        for finger in joint_names:
            col = 0
            for joint_name in finger:
                motor_index = joint_to_motor_mapping[index_jtm_mapping]
                if motor_index != -1:
                    motor = Motor(self.motors_frame, joint_name, motor_index)
                    self.motors_frame.layout().addWidget(motor, row, col)
                    self.motors.append( motor )
                    col += 1
                index_jtm_mapping += 1
            row += 1

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
                            palette.setColor(motor.revision_label.foregroundRole(), Qt.green)
                            if server_current_modified[0].strip() != server_current_modified[1].strip():
                                palette.setColor(motor.revision_label.foregroundRole(), QColor(255, 170, 23) )
                                motor.revision_label.setPalette(palette);

                            if "True" in server_current_modified[2]:
                                palette.setColor(motor.revision_label.foregroundRole(), Qt.red)
                                motor.revision_label.setText( "svn: "+ server_current_modified[1] + " [M]" )
                                motor.revision_label.setPalette(palette);
                            else:
                                motor.revision_label.setText( " svn: " + server_current_modified[1] )
                                motor.revision_label.setPalette(palette);

    def on_select_all_pressed(self):
        for motor in self.motors:
            motor.checkbox.setCheckState( Qt.Checked )

    def on_select_none_pressed(self):
        for motor in self.motors:
            motor.checkbox.setCheckState( Qt.Unchecked )

    def on_reset_motors_pressed(self):
        print("Reset motors pressed");
        self.progress_bar.reset()
        nb_motors_to_program = 0
        for motor in self.motors:
            if motor.checkbox.checkState() == Qt.Checked:
                nb_motors_to_program += 1
        if nb_motors_to_program == 0:
            QMessageBox.warning(self._widget, "Warning", "No motors selected for resetting.")
            return
        self.progress_bar.setMaximum(nb_motors_to_program)

        self.cursor = QCursor()
        self.cursor.setShape(Qt.WaitCursor)
        self._widget.setCursor(self.cursor)

        self.motor_flasher = MotorFlasher(self, nb_motors_to_program)
        self._widget.connect(self.motor_flasher, SIGNAL("finished()"), self.finished_programming_motors)
        self._widget.connect(self.motor_flasher, SIGNAL("motor_finished(QPoint)"), self.one_motor_finished)
        self._widget.connect(self.motor_flasher, SIGNAL("failed(QString)"), self.failed_programming_motors)

        self.motors_frame.setEnabled(False)
        self._widget.btn_select_all.setEnabled(False)
        self._widget.btn_select_none.setEnabled(False)
        self._widget.btn_reset_motors.setEnabled(False)

        self.motor_flasher.start()

    def one_motor_finished(self, point):
        self.progress_bar.setValue( int(point.x()) )

    def finished_programming_motors(self):
        self.motors_frame.setEnabled(True)
        self._widget.btn_select_all.setEnabled(True)
        self._widget.btn_select_none.setEnabled(True)
        self._widget.btn_reset_motors.setEnabled(True)
        self.cursor.setShape(Qt.ArrowCursor)
        self._widget.setCursor(self.cursor)

    def failed_programming_motors(self, message):
        QMessageBox.warning(self.frame, "Warning", message)

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

    def restore_settings(self, global_settings, perspective_settings):
        print "restoring settings"


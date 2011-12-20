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

from rosgui.QtBindingHelper import loadUi
from QtCore import QEvent, QObject, Qt, QTimer, Slot
from QtGui import QDockWidget, QShortcut

import roslib
roslib.load_manifest('sr_gui_controller_tuner')
import rospy

class SrGuiControllerTuner(QObject):

    def __init__(self, parent, plugin_context):
        super(SrGuiControllerTuner, self).__init__(parent)
        self.setObjectName('SrGuiControllerTuner')

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

    @Slot(str)

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


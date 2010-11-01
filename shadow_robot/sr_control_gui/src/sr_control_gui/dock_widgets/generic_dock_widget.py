
import roslib; roslib.load_manifest('sr_control_gui')

import rospy

from PyQt4 import QtCore, QtGui, Qt

class GenericDockWidget(QtGui.QDockWidget):
    def __init__(self, parent):
        QtGui.QDockWidget.__init__(self, parent=parent)
        self.shown = False
        self.setFeatures(QtGui.QDockWidget.NoDockWidgetFeatures)
        self.hide()
        
    def show_hide(self):
        if self.shown:
            self.hide()
            self.shown = False
            self.parent().show_robot_and_libraries.setText('Show Robot / Ros nodes')
            self.parent().show_robot_and_libraries.setIcon(QtGui.QIcon(self.parent().rootPath + '/images/icons/show.png'))
        else:
            self.show()
            self.shown = True
            self.parent().show_robot_and_libraries.setText('Hide Robot / Ros nodes')
            self.parent().show_robot_and_libraries.setIcon(QtGui.QIcon(self.parent().rootPath + '/images/icons/hide.png'))
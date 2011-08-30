
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

from open_gl_generic_plugin import OpenGLGenericPlugin
from utils.rostopic import RosTopicChecker
from config import Config
import math

from PyQt4 import QtCore, QtGui, Qt
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PyQt4 import QtGui
from PyQt4.QtOpenGL import *

import time, threading

from collections import deque
from sr_robot_msgs.msg import EthercatDebug

try:
    import psyco
    psyco.full()
except:
    print "psyco not found, not optimized"


class DataToDisplayChooser(QtGui.QDialog):
    """
    A popup where the user can choose which data he wants to be displayed.

    return a dictionary containing this kind of data:
         {"sensors":{"id":24, "offset": 0, "max":4000,
         "raw_color":[0.5, 0.0, 0.0],
         "scaled_color":[1.0,0.0,0],
         "color_index": 1 }}
    """

    def __init__(self, parent, current_data, index):
        """
        """
        QtGui.QDialog.__init__(self,parent)

        self.parent = parent
        self.layout = QtGui.QHBoxLayout()

        self.data_types = [ "None",
                            "sensors", "motor_data_type",
                            "which_motors", "which_motor_data_arrived",
                            "which_motor_data_had_errors",
                            "motor_data_packet_torque",
                            "motor_data_packet_misc",
                            "tactile", "idle_time_us" ]

        self.no_index = ["None","motor_data_type","which_motors",
                         "which_motor_data_arrived",
                         "which_motor_data_had_errors", "idle_time_us"]

        self.color_index = index
        try:
            self.color_index = current_data.values()[0]["color_index"]
        except:
            pass

        self.data_combo_box = QtGui.QComboBox(self)
        for d_t in self.data_types:
            self.data_combo_box.addItem(d_t)

        self.connect(self.data_combo_box, QtCore.SIGNAL('activated(int)'), self.data_changed)


        self.layout.addWidget(self.data_combo_box)

        label = QtGui.QLabel("Index: ")
        self.layout.addWidget(label)
        self.id_field = QtGui.QLineEdit(self)
        self.id_field.setText("0")

        if current_data is "None":
            self.id_field.setEnabled(False)
        elif current_data.keys()[0] in self.no_index:
            self.id_field.setEnabled(False)
        else:
            self.id_field.setText(str( current_data.values()[0]["id"] ))

        validator = QtGui.QIntValidator(0, 36, self)
        self.id_field.setValidator(validator)
        self.layout.addWidget( self.id_field )

        label = QtGui.QLabel("Max: ")
        self.layout.addWidget(label)
        self.max_field = QtGui.QLineEdit(self)
        self.max_field.setText("4000")

        if current_data is "None":
            self.max_field.setEnabled(False)
        elif current_data.keys()[0] in self.no_index:
            self.max_field.setEnabled(False)
        else:
            self.max_field.setText(str(current_data.values()[0]["max"]))

        validator = QtGui.QIntValidator(0,65535, self)
        self.max_field.setValidator(validator)
        self.layout.addWidget( self.max_field )

        self.raw_color = [0.5, 0, 0]
        self.scaled_color = [1.0, 0, 0]

        #add a combobox to easily change the color
        self.change_color_combobox = QtGui.QComboBox(self)
        self.change_color_combobox.setToolTip("Change the color for this topic")
        self.change_color_combobox.setCurrentIndex( index )
        self.change_color_combobox.setFixedWidth(45)

        self.icons = []
        for color in Config.open_gl_generic_plugin_config.colors:
            #the colors are stored as an icon + a color value
            icon = QtGui.QIcon(self.parent.parent.parent.parent.rootPath + '/images/icons/colors/'+color[0] +'.png')
            self.icons.append(icon)
            self.change_color_combobox.addItem( icon, "" )

        #add the color picker at the end of the list for the user to pick a custom color
        icon = QtGui.QIcon(self.parent.parent.parent.parent.rootPath + '/images/icons/color_wheel.png')
        self.icons.append(icon)
        self.change_color_combobox.addItem( icon, "" )

        if index > len(self.icons) - 2:
            index = len(self.icons) - 2
        self.current_icon = self.icons[ index ]
        self.connect(self.change_color_combobox, QtCore.SIGNAL('activated(int)'), self.change_color_clicked)

        #this changes the color, but we can't see it on the combobox
        self.change_color_combobox.emit(QtCore.SIGNAL("activated(int)"),
                                        self.color_index)
        self.change_color_combobox.setCurrentIndex( self.color_index )

        #this changes the selected data
        data_index = 0
        if current_data != "None":
            data_index = self.data_types.index( current_data.keys()[0] )
        self.data_combo_box.emit( QtCore.SIGNAL("activated(int)"),
                                  data_index )
        self.data_combo_box.setCurrentIndex( data_index )


        self.layout.addWidget(self.change_color_combobox)

        self.btn_box = QtGui.QDialogButtonBox(self)
        self.btn_box.setOrientation(QtCore.Qt.Horizontal)
        self.btn_box.setStandardButtons(QtGui.QDialogButtonBox.Cancel|QtGui.QDialogButtonBox.Ok)
        self.layout.addWidget(self.btn_box)
        self.setLayout(self.layout)
        self.setWindowTitle("Data To Display")

        QtCore.QObject.connect(self.btn_box, QtCore.SIGNAL("accepted()"), self.accept)
        QtCore.QObject.connect(self.btn_box, QtCore.SIGNAL("rejected()"), self.reject)
        QtCore.QMetaObject.connectSlotsByName(self)

    def change_color_clicked(self, index, const_color = None):
        # the last item is the color picker
        self.color_index = index
        if index == self.change_color_combobox.count() - 1:
            col_tmp = QtGui.QColorDialog.getColor()
            if not col_tmp.isValid():
                return
            col = [col_tmp.redF(), col_tmp.greenF(), col_tmp.blueF()]
        else:
            #the user choosed a color from the dropdown
            if const_color == None:
                col = Config.open_gl_generic_plugin_config.colors[index][1]
            else:
                col = const_color

        self.current_icon = self.icons[index]
        self.change_color(col)

    def change_color(self, rgb):
        r = rgb[0]
        g = rgb[1]
        b = rgb[2]

        self.scaled_color = [r, g, b]

        #create a darker version of the raw color
        r *= 0.5
        g *= 0.5
        b *= 0.5
        self.raw_color = [r,g,b]

    def data_changed(self, index):
        name = self.data_types[index]
        if name in self.no_index:
            self.id_field.setEnabled(False)
        else:
            self.id_field.setEnabled(True)
        self.max_field.setEnabled(True)
        if name is "None":
            self.max_field.setEnabled(False)


    def getValues(self):
        name = str( self.data_combo_box.currentText() )
        index = self.id_field.text().toInt()[0]
        maximum = self.max_field.text().toInt()[0]

        if name is "None":
            return [None, {"None":None}]

        if name in self.no_index:
            return [ self.current_icon,
                     {name:{ "offset":0, "max": maximum,
                             "raw_color": self.raw_color,
                             "scaled_color": self.scaled_color,
                             "color_index": self.color_index}}]

        return [ self.current_icon,
                 {name:{"id": index,
                        "offset":0, "max": maximum,
                        "raw_color": self.raw_color,
                        "scaled_color": self.scaled_color,
                        "color_index": self.color_index}}]

class DataToDisplayWidget(QtGui.QFrame):
    def __init__(self, parent, index):
        self.parent = parent
        self.index = index
        QtGui.QFrame.__init__(self)
        self.layout = QtGui.QHBoxLayout()

        self.btn_choose_data = QtGui.QPushButton()
        self.btn_choose_data.setText("Data")
        self.connect(self.btn_choose_data, QtCore.SIGNAL('clicked()'), self.choose_data)
        self.layout.addWidget(self.btn_choose_data)

        self.label_value = QtGui.QLabel()
        self.layout.addWidget(self.label_value)

        #add a button to add a subscribe topic frame
        self.add_subscribe_topic_btn = QtGui.QPushButton()
        self.add_subscribe_topic_btn.setText('+')
        self.add_subscribe_topic_btn.setToolTip("Add a new topic to plot.")
        self.add_subscribe_topic_btn.setFixedWidth(30)
        self.connect(self.add_subscribe_topic_btn, QtCore.SIGNAL('clicked()'),self.add_subscribe_topic_clicked)
        self.layout.addWidget(self.add_subscribe_topic_btn)

        #add a button to remove the current subscribe topic frame
        if len(self.parent.data_to_display_widgets) > 0:
            self.remove_topic_btn = QtGui.QPushButton()
            self.remove_topic_btn.setText('-')
            self.remove_topic_btn.setToolTip("Remove this topic.")
            self.remove_topic_btn.setFixedWidth(30)
            self.connect(self.remove_topic_btn, QtCore.SIGNAL('clicked()'),self.remove_topic_clicked)
            self.layout.addWidget(self.remove_topic_btn)

        self.setLayout(self.layout)

    def choose_data(self):
        data_chooser = DataToDisplayChooser(self, self.parent.data_to_display[self.index], self.index)
        if data_chooser.exec_():
            data = data_chooser.getValues()
            if data[1].keys()[0] is "None":
                self.btn_choose_data.setText("None")
                self.parent.data_to_display[self.index] = "None"
            else:
                if "id" in data[1].values()[0].keys():
                    self.btn_choose_data.setText(data[1].keys()[0] + "["+str(data[1].values()[0]["id"])+"]")
                else:
                    self.btn_choose_data.setText(data[1].keys()[0])
                self.btn_choose_data.setIcon(data[0])
                self.parent.data_to_display[self.index] = data[1]

    def add_subscribe_topic_clicked(self):
        self.parent.add_data_to_display_widget()
        Qt.QTimer.singleShot(0, self.parent.btn_frame.adjustSize)

    def remove_topic_clicked(self):
        self.parent.data_to_display.pop(self.index)
        self.parent.data_to_display_widgets.remove(self)
        self.setParent(None)

        self.btn_choose_data.setParent(None)
        self.label_value.setParent(None)
        self.add_subscribe_topic_btn.setParent(None)
        self.remove_topic_btn.setParent(None)

        #refresh the indexes
        for i,widget in enumerate(self.parent.data_to_display_widgets):
            widget.subscriber_index = i

        Qt.QTimer.singleShot(0, self.parent.btn_frame.adjustSize)
        del self

    def set_value(self, value):
        if value != None:
            txt = ""
            txt += str(value)
            txt += " / "
            txt += "0x%0.4X" % value
            self.label_value.setText(txt)


class DataSet(threading.Thread):
    default_color = QtCore.Qt.black

    def __init__(self, parent):
        threading.Thread.__init__(self)
        self.parent = parent
        self.subscriber = None
        self.enabled = False

        self.points = []
        self.init_dataset()
        self.scaled_color = []
        self.raw_color = []
        self.color_index = 0
        self.change_color([255,0,0])

    def run(self):
        self.subscriber = rospy.Subscriber("/debug_etherCAT_data", EthercatDebug, self.msg_callback)

    def msg_callback(self, msg):
        """
        Received a message on the topic. Adding the new point to
        the queue.
        """
        #received message from subscriber at index: update the last
        # point and pop the first point
        if self.parent.paused:
            return

        self.parent.mutex.acquire()
        new_data = { "sensors": msg.sensors,
                     "motor_data_type": msg.motor_data_type,
                     "which_motors": msg.which_motors,
                     "which_motor_data_arrived": msg.which_motor_data_arrived,
                     "which_motor_data_had_errors": msg.which_motor_data_had_errors,
                     "motor_data_packet_torque": msg.motor_data_packet_torque,
                     "motor_data_packet_misc": msg.motor_data_packet_misc,
                     "tactile": msg.tactile,
                     "idle_time_us": msg.idle_time_us }
        self.points.append(new_data)
        self.points.popleft()
        self.parent.mutex.release()

    def init_dataset(self):
        tmp = [None]* self.parent.data_points_size
        self.points = deque(tmp)

    def change_color(self, rgb):
        r = rgb[0]
        g = rgb[1]
        b = rgb[2]

        self.scaled_color = [r, g, b]

        #create a darker version of the raw color
        r *= 0.5
        g *= 0.5
        b *= 0.5
        self.raw_color = [r,g,b]

    def get_raw_color(self):
        return self.raw_color

    def get_scaled_color(self):
        return self.scaled_color

    def close(self):
        self.subscriber.unregister()
        self.subscriber = None

class SensorScope(OpenGLGenericPlugin):
    """
    Plots some chosen debug values.
    """
    name = "Sensor Scope"
    mouse_wheel_zoom = 1.0

    def __init__(self):
        OpenGLGenericPlugin.__init__(self, self.paint_method, self.right_click_method, self.left_click_method,
                                     self.mouse_wheel_method)
        self.data_points_size = self.open_gl_widget.number_of_points

        # this is used to go back in time
        self.display_frame = 0

        self.data_to_display = ["None"]

        self.control_layout = QtGui.QVBoxLayout()

        #this is a line used to display the data intersecting it
        self.line_x = 10
        self.line = [[self.line_x, 0],
                     [self.line_x, self.open_gl_widget.height]]
        self.line_color = [1.0,1.0,1.0]

        self.mutex = threading.Lock()

        self.btn_frame = QtGui.QFrame()
        self.btn_frame_layout = QtGui.QHBoxLayout()
        # add a button to play/pause the display
        self.play_btn = QtGui.QPushButton()
        self.play_btn.setFixedWidth(30)
        self.play_btn.setToolTip("Play/Pause the plots")
        self.btn_frame.connect(self.play_btn, QtCore.SIGNAL('clicked()'), self.button_play_clicked)
        self.btn_frame_layout.addWidget(self.play_btn)

        self.btn_frame.setFixedWidth(80)
        self.btn_frame.setFixedHeight(50)
        self.btn_frame.setLayout(self.btn_frame_layout)

        self.control_layout.addWidget(self.btn_frame)

        self.data_to_display_widgets = []
        self.add_data_to_display_widget()

        self.time_slider = QtGui.QSlider(QtCore.Qt.Horizontal)
        self.time_slider.setMinimum(0)
        self.time_slider.setMaximum(self.data_points_size - self.open_gl_widget.number_of_points_to_display)
        self.time_slider.setInvertedControls(True)
        self.time_slider.setInvertedAppearance(True)
        self.time_slider.setEnabled(False)
        self.frame.connect(self.time_slider, QtCore.SIGNAL('valueChanged(int)'), self.time_changed)
        self.layout.addWidget(self.time_slider)

        self.paused = False

    def resized(self):
        #we divide by 100 because we want to scroll 100 points per 100 points
        self.time_slider.setMaximum((self.data_points_size - self.open_gl_widget.number_of_points_to_display)/100 - 1)

    def activate(self):
        self.data_set = DataSet(self)
        self.data_set.start()
        OpenGLGenericPlugin.activate(self)
        self.play_btn.setIcon(QtGui.QIcon(self.parent.parent.rootPath + '/images/icons/pause.png'))
        self.control_frame.setLayout(self.control_layout)

    def right_click_method(self, x):
        #right click for time traveling
        # only if paused
        if self.paused:
            self.display_frame += (x - self.open_gl_widget.last_right_click_x)
            if self.display_frame < 0:
                self.display_frame = 0
            if self.display_frame >= self.data_points_size:
                self.display_frame = self.data_points_size - 1

            self.open_gl_widget.last_right_click_x = x

            self.paint_method(self.display_frame)


    def left_click_method(self, x):
        """
        Moves the white line.
        """
        self.line_x = x
        self.line = [[self.line_x, 0],
                     [self.line_x, self.open_gl_widget.height]]

    def mouse_wheel_method(self, delta, x, y):
        #delta is a multiple of 120
        self.mouse_wheel_zoom += (delta / 1000.)
        self.mouse_wheel_zoom = max(self.mouse_wheel_zoom, 0.1)

        self.open_gl_widget.setToolTip("zoom is set to : "+str(self.mouse_wheel_zoom))

        for data_tmp in self.data_to_display:
            if data_tmp is "None":
                continue

            data_tmp.values()[0]["max"] /= self.mouse_wheel_zoom

    def paint_method(self, display_frame = 0):
        '''
        Drawing routine: this function is called periodically.

        @display_frame: the frame to display: we have more points than we display.
                        By default, we display the latest values, but by playing
                        with the time_slider we can then go back in time.
        '''
        times = []
        times.append(["start_time",time.time()])

        if self.paused:
            display_frame = self.display_frame

        display_points = []
        colors = []

        self.mutex.acquire()
        try:
            #we want to keep the average of the last 50 points of
            #  the raw data in the middle of the screen
            #first we remove the data containing None from the 50 last points
            for data_to_display in self.data_to_display:
                if data_to_display is "None":
                    continue
                data_tmp = data_to_display.items()[0]
                data_name = data_tmp[0]
                data_param = data_tmp[1]

                last_50_points = []
                index = (self.data_points_size - display_frame -1)
                iteration = 0
                while len(last_50_points) < 50:
                    if self.data_set.points[index] != None:
                        value = 0
                        if 'id' in data_param.keys():
                            value = self.data_set.points[index][data_name][data_param['id']]
                        else:
                            if data_name is "motor_data_type":
                                value = self.data_set.points[index][data_name].data
                            else:
                                value = self.data_set.points[index][data_name]
                        last_50_points.append( value )
                    index -= 1
                    iteration += 1
                    #searched the last 200 points
                    if iteration == 200:
                        break

                if len(last_50_points) != 0:
                    offset = self.open_gl_widget.height/2 - int(float(sum(last_50_points))/len(last_50_points))
                else:
                    offset = 0
                data_to_display[data_name]["offset"] = offset

            times.append(["last 50",time.time()])


            for display_index in range(0, self.open_gl_widget.number_of_points_to_display):
                data_index = self.display_to_data_index(display_index, display_frame)
                if self.data_set.points[data_index] != None:
                    for data_tmp in self.data_to_display:
                        if data_tmp is "None":
                            continue
                        data_tmp = data_tmp.items()[0]
                        data_name = data_tmp[0]
                        data_param = data_tmp[1]
                        value = 0
                        if 'id' in data_param.keys():
                            value = self.data_set.points[data_index][data_name][data_param['id']]
                        else:
                            if data_name is "motor_data_type":
                                value = self.data_set.points[data_index][data_name].data
                            else:
                                value = self.data_set.points[data_index][data_name]
                        # add the raw data
                        colors.append(data_param['raw_color'])
                        display_points.append([display_index, value + data_param['offset'] ])
                        #also add the scaled data
                        colors.append(data_param['scaled_color'])
                        display_points.append([display_index, self.scale_data(value, data_param['max'])] )

            times.append(["compute",time.time()])

            glEnableClientState(GL_VERTEX_ARRAY)
            glEnableClientState(GL_COLOR_ARRAY)
            glClear(GL_COLOR_BUFFER_BIT)
            #glDrawArrays(GL_LINE_STRIP, 0, len(display_points))
            #glPointSize(10)
            #glEnable(GL_POINT_SMOOTH)
            #glEnable(GL_BLEND)

            times.append(["open gl init",time.time()])

            glColorPointerf(colors)
            glVertexPointerf(display_points)
            glDrawArrays(GL_POINTS, 0, len(display_points))

            times.append(["open gl points",time.time()])

            #draw the vertical line
            glColorPointerf(self.line_color)
            glVertexPointerf(self.line)
            glDrawArrays(GL_LINES, 0, len(self.line))

            times.append(["open gl draw line",time.time()])


            self.compute_line_intersect(display_frame)
            self.mutex.release()

            times.append(["compute line intersect ",time.time()])

            glDisableClientState(GL_VERTEX_ARRAY)
            glDisableClientState(GL_COLOR_ARRAY)
            glFlush()

            times.append(["open gl flush",time.time()])

            if False:
                print "------"
                time_max = ["",0.0]
                for i in range(1,len(times)):
                    print times[i][0], "->",round(times[i][1] - times[i-1][1] , 3),"/",
                    if ( times[i][1] - times[i-1][1] ) > time_max[1]:
                        time_max = [ times[i][0], times[i][1] - times[i-1][1] ]
                print ""
                print "    -> TOTAL time: ", round(times[-1][1] - times[0][1] , 3), " max: ", time_max[0], "->",round( time_max[1], 3 )

        except:
            if self.mutex.locked():
                self.mutex.release()
            print "ERROR"
            pass
        self.open_gl_widget.update()

    def add_data_to_display_widget(self):
        index = len(self.data_to_display_widgets)
        self.data_to_display.append("None")
        tmp_widget = DataToDisplayWidget(self, index)
        self.control_layout.addWidget( tmp_widget )
        self.data_to_display_widgets.append( tmp_widget )

    def time_changed(self, value):
        self.display_frame = value*100
        self.paint_method(value)

    def button_play_clicked(self):
        #lock mutex here
        if not self.paused:
            self.paused = True
            self.play_btn.setIcon(QtGui.QIcon(self.parent.parent.rootPath + '/images/icons/play.png'))
            self.time_slider.setEnabled(True)
        else:
            self.paused = False
            self.play_btn.setIcon(QtGui.QIcon(self.parent.parent.rootPath + '/images/icons/pause.png'))
            self.time_slider.setEnabled(False)

    def display_to_data_index(self, display_index, display_frame):
        # we multiply display_frame by a 100 because we're scrolling 100 points per 100 points
        data_index = self.data_points_size - self.open_gl_widget.number_of_points_to_display + display_index - (display_frame)
        return data_index

    def scale_data(self, data, data_max = 65536):
        scaled_data = (data * self.open_gl_widget.height) / data_max
        return scaled_data

    def compute_line_intersect(self, display_frame):
        data_index = self.display_to_data_index(self.line_x, display_frame)
        #update the value in the label
        for widget,data_to_display in zip(self.data_to_display_widgets,self.data_to_display):
            if data_to_display is "None":
                continue
            data_tmp = data_to_display.items()[0]
            data_name = data_tmp[0]
            data_param = data_tmp[1]
            if self.data_set.points[data_index] != None:
                value = 0
                if 'id' in data_param.keys():
                    value = self.data_set.points[data_index][data_name][data_param['id']]
                else:
                    if data_name is "motor_data_type":
                        value = self.data_set.points[data_index][data_name].data
                    else:
                        value = self.data_set.points[data_index][data_name]
            widget.set_value(value)

    def close(self):
        OpenGLGenericPlugin.on_close(self)
        self.data_set.close()
        self.data_set.joint()


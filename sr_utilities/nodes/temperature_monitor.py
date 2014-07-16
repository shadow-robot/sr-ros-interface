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

import rospy
from diagnostic_msgs.msg import DiagnosticArray

import curses
import traceback

CASE_WIDTH = 20
CASE_HEIGHT = 1
JOINT_NAMES = ["FFJ0", "FFJ3", "FFJ4",
               "MFJ0", "MFJ3", "MFJ4",
               "RFJ0", "RFJ3", "RFJ4",
               "LFJ0", "LFJ3", "LFJ4",
               "THJ1", "THJ2", "THJ3", "THJ4", "THJ5",
               "WRJ1", "WRJ2"]
COOL = 50
WARM = 55

class Joint(object):
    def __init__(self, screen, joint_name, x, y):
        self.screen = screen
        self.x = x
        self.y = y
        self.joint_name = joint_name
        self.temperature = -1

        #self.window = self.screen.subwin(1, 15, self.y+1, self.x + 1)
        self.refresh()


    def set_temperature(self, temperature):
        self.temperature = temperature
        self.refresh()

    def refresh(self):
        self.screen.addstr(self.y+1, self.x+1, self.joint_name)

        if self.temperature == -1: #joint not found
            self.screen.addstr(self.y + 1, self.x + 6, "X", curses.color_pair(4) )
        elif self.temperature < COOL:
            self.screen.addstr(self.y + 1, self.x+6, str(self.temperature), curses.color_pair(1) )
        elif self.temperature < WARM:
            self.screen.addstr(self.y + 1, self.x + 6, str(self.temperature), curses.color_pair(2) )
        else:
            self.screen.addstr(self.y + 1, self.x + 6, str(self.temperature), curses.color_pair(3) )
        #self.window.refresh()#0,0,0,0,1,15)

class TemperatureMonitor(object):
    MAX_X = 17
    MAX_Y = 21

    def __init__(self, screen = None):
        try:
            curses.curs_set(0)
        except:
            pass
        self.screen = screen
        self.pad = curses.newpad(self.MAX_Y,self.MAX_X)
        self.pad_pos_x_ = 0
        self.pad_pos_y_ = 0
        self.pad.border(0)
        self.joint_monitors = {}

        curses.init_pair(4, curses.COLOR_WHITE, curses.COLOR_BLACK)
        curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_GREEN)
        curses.init_pair(2, curses.COLOR_BLACK, curses.COLOR_YELLOW)
        curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_RED)

        for index,joint_name in enumerate(JOINT_NAMES):
            begin_x = 0
            begin_y = CASE_HEIGHT*index

            self.joint_monitors[joint_name] = Joint(self.pad, joint_name, begin_x, begin_y)

        self.diag_sub_ = rospy.Subscriber("/diagnostics", DiagnosticArray, self.diag_cb_)
        self.resize_()

        while True:
            event = self.pad.getch()
            if event == ord("q"): break
            elif event == curses.KEY_RESIZE:
                self.resize_()

            elif event == ord("s"):
                self.pad_pos_y_ += 1
                self.refresh_()
            elif event == ord("w"):
                self.pad_pos_y_ -= 1
                self.refresh_()
            elif event == ord("a"):
                self.pad_pos_x_ -= 1
                self.refresh_()
            elif event == ord("d"):
                self.pad_pos_x_ += 1
                self.refresh_()

    def diag_cb_(self, msg):
        for status in msg.status:
            for joint in JOINT_NAMES:
                if joint in status.name:
                    for value in status.values:
                        if value.key == "Temperature":
                            self.joint_monitors[joint].set_temperature(round(float(value.value), 1))
                            break
                    break
        self.resize_()

    def resize_(self):
        self.pad_pos_x_ = 0
        self.pad_pos_y_ = 0
        self.refresh_()

    def refresh_(self):
        y,x = self.screen.getmaxyx()
        self.pad_pos_x_ = min(max(self.pad_pos_x_, 0), self.MAX_X - 1)
        self.pad_pos_y_ = min(max(self.pad_pos_y_, 0), self.MAX_Y - 1)
        self.pad.refresh(self.pad_pos_y_, self.pad_pos_x_, 0,0, y - 1, x -1)
        self.pad.border(0)
        for monitor in self.joint_monitors.values():
            monitor.refresh()


if __name__ == '__main__':
    rospy.init_node("temperature_monitor", anonymous=True)
    try:
        curses.wrapper(TemperatureMonitor)
    except:
        traceback.print_exc()
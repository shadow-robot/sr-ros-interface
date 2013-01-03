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

import roslib; roslib.load_manifest("sr_utilities")
import rospy

import curses, traceback

CASE_WIDTH = 20
CASE_HEIGHT = 1
JOINT_NAMES = ["FFJ0", "FFJ3", "FFJ4", 
               "MFJ0", "MFJ3", "MFJ4",
               "RFJ0", "RFJ3", "RFJ4",
               "LFJ0", "LFJ3", "LFJ4",
               "THJ1", "THJ2", "THJ3", "THJ4", "THJ5",
               "WRJ1", "WRJ2"]
COOL = 5
WARM = 10

class Joint(object):
    def __init__(self, screen, joint_name, x, y):
        self.screen = screen
        self.x = x
        self.y = y
        self.joint_name = joint_name
        
        self.window = self.screen.subwin(1, 15, self.y+1, self.x + 1)
        self.window.addstr(0, 0, joint_name)
                                
    def set_temperature(self, temperature):
        if temperature < COOL:
            self.window.addstr(0, 6, str(temperature), curses.color_pair(1) )
        elif temperature < WARM:
            self.window.addstr(0, 6, str(temperature), curses.color_pair(2) )
        else:
            self.window.addstr(0, 6, str(temperature), curses.color_pair(3) )
        
class TemperatureMonitor(object):    
    def __init__(self, screen):
        curses.curs_set(0)
        self.screen = screen
        self.screen.border(0)
        self.joint_monitors = {}
        
        curses.init_pair(4, curses.COLOR_WHITE, curses.COLOR_BLACK)
        curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_GREEN)
        curses.init_pair(2, curses.COLOR_BLACK, curses.COLOR_MAGENTA)
        curses.init_pair(3, curses.COLOR_BLACK, curses.COLOR_RED)
        
        for index,joint_name in enumerate(JOINT_NAMES):
            begin_x = 0
            begin_y = CASE_HEIGHT*index

            self.joint_monitors[joint_name] = Joint(self.screen, joint_name, begin_x, begin_y)
            self.joint_monitors[joint_name].set_temperature(index)
                  
        while True:
            event = self.screen.getch()
            if event == ord("q"): break
                                
if __name__ == '__main__':
    rospy.init_node("temperature_monitor", anonymous=True)
    try:
        curses.wrapper(TemperatureMonitor)
    except:
        traceback.print_exc()
    
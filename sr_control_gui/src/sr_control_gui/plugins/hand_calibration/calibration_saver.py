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

import yaml
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

class CalibrationLoader(object):
    def __init__(self):
        pass

    def get_settings(self, calibration_parameter = "sr_calibrations"):
        param_dict = rospy.get_param(calibration_parameter)
        return param_dict

class CalibrationSaver(object):
    def __init__(self, file_path):
        self.path = file_path

    def save_settings(self, calibration):
        f = open(self.path,'r')
        document = ""
        for line in f.readlines():
            document += line
        f.close()

        yaml_config = yaml.load(document)

        for index,old_joint in enumerate(yaml_config["sr_calibrations"]):
            old_joint_name = old_joint[0]

            for new_joint in calibration:
                new_joint_name = new_joint[0][0]
                new_joint_calibration = new_joint[1]

                if new_joint_name == old_joint_name:
                    yaml_config["sr_calibrations"][index][1] = new_joint_calibration

        f = open(self.path, 'w')
        f.write("#Generated From sr_control_gui, Hand Calibration plugin.\n")
        f.write("\n")
        f.write("sr_calibrations: [\n")
        for index in range(0, len(yaml_config["sr_calibrations"]) - 1):
            finger = yaml_config["sr_calibrations"][index]
            f.write( str(finger) + ",\n" )

        finger = yaml_config["sr_calibrations"][index + 1]
        f.write( str(finger) + "\n" )
        f.write("]\n")
        f.close()

if __name__ == '__main__':
    calibration_saver = CalibrationSaver("/code/Projects/ROS_interfaces/etherCAT_workspace/shadow_robot_ethercat/sr_robot_lib/config/calibration.yaml")
    calibration_saver.save_settings([[["FFJ3"], [[0.0,0.0], [1.0,1.0]]]])
[['THJ1', [[-1.0, 0.0], [-1.0, 22.5]]]]

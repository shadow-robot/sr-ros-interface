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

class PidLoader(object):
    def __init__(self):
        pass

    def get_settings(self, controller_topic):
        param_dict = {}
        if len(controller_topic) == 2:
            tmp_dict = rospy.get_param(controller_topic[0])
            for item in tmp_dict.items():
                param_dict["position_pid/"+item[0]] = item[1]

            tmp_dict = rospy.get_param(controller_topic[1])
            for item in tmp_dict.items():
                param_dict["velocity_pid/"+item[0]] = item[1]
        else:
            param_dict = rospy.get_param(controller_topic)
        return param_dict

class PidSaver(object):
    def __init__(self, file_path):
        self.path = file_path

    def save_settings(self, param_path, parameters_dict):
        f = open(self.path,'r')
        document = ""
        for line in f.readlines():
            document += line
        f.close()

        yaml_config = yaml.load(document)
        if len(param_path) == 2:
            for item in parameters_dict.items():
                if "position_pid/" in item[0]:
                    yaml_config[param_path[0]]["position_pid"][item[0].split("position_pid/")[1]] = item[1]
                elif "velocity_pid/" in item[0]:
                    yaml_config[param_path[0]]["velocity_pid"][item[0].split("velocity_pid/")[1]] = item[1]
                else:
                    yaml_config[param_path[0]][param_path[1]][item[0]] = item[1]
        elif len(param_path) == 1:
            for item in parameters_dict.items():
                yaml_config[param_path][param_path][item[0]] = item[1]
        else:
            print "wrong size for the param_path array"

        full_config_to_write = yaml.dump(yaml_config, default_flow_style=False)

        f = open(self.path, 'w')
        f.write(full_config_to_write)
        f.close()

if __name__ == '__main__':
    pid_saver = PidSaver("/code/Projects/ROS_interfaces/sr-ros-interface/palm_edc/shadow_robot/sr_hand_palm_edc/sr_edc_controller_configuration/sr_edc_mixed_position_velocity_joint_controllers.yaml")
    pid_saver.save_settings(["sh_wrj2_mixed_position_velocity_controller","pid"], {"d":1.0})

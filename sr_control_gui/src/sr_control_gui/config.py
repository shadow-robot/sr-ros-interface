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
"""
File containing all the configuration parameters.
"""

class OpenGLGenericPluginConfig(object):
    number_of_points = 1000000
    colors = [["red", [1.0,0,0]],
              ["green", [0,1.0,0]],
              ["blue", [0,0,1.0]],
              ["light_blue", [0.51,0.75,1.]],
              ["orange", [1.,0.66,0]],
              ["yellow", [1.,0.92,0.06]],
              ["purple", [0.85,0.13,0.82]],
              ["pink", [1.,0.51,0.80]]
              ]

class GenericPluginConfig(object):
    dependencies = []

class GenericRosNodeConfig(object):
    name = ""
    list_of_nodes = []
    start_cmd = "roslaunch mypkg mylaunchfile.launch"
    stop_cmd = "rosnode kill " + " ".join(list_of_nodes)
    status_cmd = "rosnode list"

class ShadowHandConfig(GenericRosNodeConfig):
    name = "Shadow Hand"
    list_of_nodes = ["/shadowhand"]
    start_cmd = "roslaunch sr_hand srh_motor.launch"
    stop_cmd = "rosnode kill " + " ".join(list_of_nodes)
    status_cmd = "rosnode list"

class ShadowHandPluginConfig(GenericPluginConfig):
    dependencies = ["Shadow Hand"]

class ShadowArmHandConfig(GenericRosNodeConfig):
    name = "Shadow Hand and Arm"
    list_of_nodes = ["/shadowarm",
                      "/shadowhand",
                      "/srh_robot_state_publisher_pos",
                      "/srh_robot_state_publisher_target",
                      "/fixed_frame_pos_pub_arm",
                      "/fixed_frame_target_pub_arm",
                      "/link_hand_arm_pos",
                      "/link_hand_arm_target",
                      "/robot_state_publisher_pos_arm",
                      "/robot_state_publisher_target_arm"]
    start_cmd = "roslaunch sr_hand sr_arm_motor.launch"
    stop_cmd = "rosnode kill " + " ".join(list_of_nodes)
    status_cmd = "rosnode list"

class ShadowArmConfig(GenericRosNodeConfig):
    name = "Shadow Arm"
    list_of_nodes = ["/shadowarm",
                     "/fixed_frame_pos_pub_arm",
                      "/fixed_frame_target_pub_arm",
                      "/robot_state_publisher_pos_arm",
                      "/robot_state_publisher_target_arm"]
    start_cmd = "roslaunch sr_hand real_arm_motor.launch"
    stop_cmd = "rosnode kill " + " ".join(list_of_nodes)
    status_cmd = "rosnode list"

class ShadowArmPluginConfig(GenericPluginConfig):
    dependencies = ["Shadow Arm"]

class SrObjectSelectionConfig(GenericPluginConfig):
    name = "Object Selection Plugin"
    list_of_nodes = ["/tabletop_node"]

class CybergloveConfig(GenericRosNodeConfig):
    name = "Cyberglove"
    list_of_nodes = ["/cyberglove"]
    start_cmd = "roslaunch cyberglove cyberglove.launch"
    stop_cmd = "rosnode kill " + " ".join(list_of_nodes)
    status_cmd = "rosnode list"

class CybergloveRemapperConfig(GenericRosNodeConfig):
    name = "Glove Remapper"
    list_of_nodes = ["/cyberglove_remapper"]
    start_cmd = "roslaunch sr_remappers remapper_glove.launch"
    stop_cmd = "rosnode kill " + " ".join(list_of_nodes)
    status_cmd = "rosnode list"

class CybergloveGenericPluginConfig(GenericPluginConfig):
    dependencies = ["Cyberglove"]

class RobotCodeConfig(object):
    name = "Robot Hand"
    start_cmd = "sudo /etc/init.d/robot start"
    stop_cmd = "sudo /etc/init.d/robot stop"
    status_cmd = "ls /proc/robot/ | wc -l"


class Config(object):

    #ROS nodes config
    library_refresh_rate = 0.5
    library_timeout = 10
    library_shadowhand = ShadowHandConfig()
    library_shadowarm = ShadowArmConfig()
    library_shadow_arm_hand = ShadowArmHandConfig()
    library_cyberglove = CybergloveConfig()
    library_cyberglove_remapper = CybergloveRemapperConfig()

    #Robot code config
    robot_code = RobotCodeConfig()

    #Plugins config
    generic_plugin_config = GenericPluginConfig()
    shadowhand_plugin_config = ShadowHandPluginConfig()
    cyberglove_generic_plugin_config = CybergloveGenericPluginConfig()
    shadow_arm_plugin_config = ShadowArmPluginConfig()
    sr_object_selection_config = SrObjectSelectionConfig()

    #Other GUI config
    main_widget_refresh_rate = 0.5

    #Open GL plugins
    open_gl_generic_plugin_config = OpenGLGenericPluginConfig()

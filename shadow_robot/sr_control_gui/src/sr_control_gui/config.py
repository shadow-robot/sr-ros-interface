#!/usr/bin/env python
"""
File containing all the configuration parameters. 
"""

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

class ShadowArmPluginConfig(GenericPluginConfig):
    dependencies = ["Shadow Hand and Arm"]

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


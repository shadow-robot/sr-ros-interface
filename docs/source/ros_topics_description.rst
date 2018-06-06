
# Hand E ROS Kinetic Topics

This document explains the topics used for the Shadow Robot Humanoid Hand (Hand E), which can be run in either Torque control or Position control.
Assume that all the topics are read only unless specified otherwise.

Definitions used :
Hand = Humanoid Hand (Hand E)
Host = Host Computer, which is controlling the Hand

Contents:
Using rostopic
Trajectory Control
    Moveit! Topics
    RViz Topics

## Using rostopic

To see at what rate a topic is published use :
rostopic hz <ROS_TOPIC>

To see which nodes are publishing and subscribing to topics, as well as the topic message type use :
rostopic info <ROS_TOPIC>

Where <ROS_TOPIC> is the topic under scrutiny.

For additional information on ROS topics see : http://wiki.ros.org/rostopic

## Trajectory Control

The following topics described are active using a real Hand E and launching :
roslaunch sr_ethercat_hand_config sr_rhand.launch


This rqt_graph shows the flow of topics between nodes whilst running : https://drive.google.com/file/d/1qql0WbgprA80IwDrDELh8RsrF1o3i266/view?usp=sharing

**_/cal_sh_rh_*/calibrated_**
**_/calibrated_**
These topics are used during the Hand startup routine to make sure that Hand is calibrated./calibrated
These topics are used during the Hand startup routine to make sure that Hand is calibrated.

An empty message is published to the /cal_sh_rh_*/calibrated topics for each joint when they are calibrated. The /calibrate_sr_edc node subscribes to these topics and when all of them have had a empty message published to them, it publishes True to the /calibrated topic. Before empty messages have been received by all the joints it publishes False to the /calibrated topic.


**_/diagnostics_**
**_/diagnostics_agg_**
**_/diagnostics_toplevel_state
These topics update at 2 Hz with information on each joints Temperature, Current, Measured effort and Command effort, as well as information about the EtherCat devices and firmware version, and contain all the diagnostics information that gets published from the fh_driver and fh_safety_checks nodes.

It should not be necessary to publish to these topic from a terminal.

/diagnostics is uncategorized, where the /diagnostics_agg is categorizes using the diagnostic_analyzer.yaml : https://github.com/shadow-robot/fh_config/blob/kinetic-devel/fh_config/diagnostic_analyzer.yaml

You can see the output from these topics in rqt : Plugins->Robot Tools->Diagnostics Viewer


**_/joint_0s/joint_states_**

This topic is not currently used and may be soon removed.


**_/joint_states_**
This topic is read-only and updates at 100 Hz with the name, position, velocity and effort values of all joints in a Hand.

Example topic message :
name: [rh_FFJ1, rh_FFJ2, rh_FFJ3, rh_FFJ4, rh_LFJ1, rh_LFJ2, rh_LFJ3, rh_LFJ4, rh_LFJ5,
rh_MFJ1, rh_MFJ2, rh_MFJ3, rh_MFJ4, rh_RFJ1, rh_RFJ2, rh_RFJ3, rh_RFJ4, rh_THJ1,
rh_THJ2, rh_THJ3, rh_THJ4, rh_THJ5, rh_WRJ1, rh_WRJ2]
position: [1.279751244673038, 1.7231505348398373, 1.2957917583498741, -0.00406710173435502, 0.054689233814909366, 1.253488840949725, 1.5395435039130654, 0.02170017906073821, 0.1489674305718295, 1.08814400717011, 1.638917596069165, 1.4315445985097324, 0.00989364236002074, 1.2257618075487349, 1.8331224739256338, 1.2888368284819698, -0.13269012433948385, 0.14435534682895756, 0.6980816915624072, 0.18782898954368935, 1.124295322901818, 0.21905854304869088, -0.048455186771971595, -0.0032803323337213066]
velocity: [-7.484333985952662e-06, -7.484333985952662e-06, 0.0023735860019749185, 0.00062181267775619, -0.0005871136552505063, -0.0005871136552505063, 0.0020967687295392933, 0.0001739028157522596, 0.0004985252400775274, -9.485516545601461e-06, -9.485516545601461e-06, -0.0007068752456452666, -0.0012475428276090576, 0.0008426052935621657, 0.0008426052935621657, 0.001237001167977189, -0.0026444893567459573, 0.0025260047430310925, -0.0003217106977882921, 6.159570145597239e-05, -0.0023454723015513593, 0.0009436399232442155, 0.00017469681801687975, -4.900148416020751e-05]
effort: [-1.3660655058510802, -1.3660655058510802, -2.030169817308198, -1.9577332816789155, 0.0, 0.0, -17.29928766980003, -1.5006516553524243, -1.8579749510438912, -1.504877130092884, -1.504877130092884, -0.3374653182042338, -1.6492254479379729, -8.476660697182016, -8.476660697182016, -3.3867013328219056, -2.3404145772688683, -0.7688013735971971, 11.02319645071454, 0.8482082620071664, 0.08818910881575533, 1.127772119947565, -2.2344970991165316, -3.5544023107705667]


**_/rh/biotac_*_**
These topics are read-only and update at 100 Hz with data from the biotac sensors, which comprises their pressure, temperature and electrode resistance. This topic is published from the /biotac_republisher node which receives this data from the driver via the /rh/tactile topic. For further information about the biotacts, refer to their documentation : https://www.syntouchinc.com/wp-content/uploads/2016/12/BioTac_SP_Product_Manual.pdf

Example topic message :
pac0: 2056
pac1: 2043
pdc: 2543
tac: 2020
tdc: 2454
electrodes: [2512, 3062, 2404, 2960, 2902, 2382, 2984, 138, 2532, 2422, 2809, 3167, 2579, 2950, 2928, 2269, 2966, 981, 2374, 2532, 3199, 3152, 3155, 3033]


**_/rh/debug_etherCAT_data_**
This topic updates at 800 Hz with information for debugging. It is similar to the driver state topic used for Hand H which shows data from the Hand as it is received.

Example topic message :
sensors: [1559, 1403, 2836, 1860, 1295, 1555, 3077, 2237, 979, 1270, 2848, 1921, 3150, 1914, 3107, 1817, 2669, 2430, 2194, 2542, 2560, 3768, 645, 1099, 3156, 1636, 26, 27, 28, 29, 30, 31, 0, 0, 0, 0, 0]
motor_data_type:
data: 2
which_motors: 0
which_motor_data_arrived: 345429
which_motor_data_had_errors: 0
motor_data_packet_torque: [1, -3, -4, -1, 0, -23, 0, 1, -10, -1]
motor_data_packet_misc: [-3, -2, 0, -3, -4, 0, 0, -4, 0, -3]
tactile_data_type: 5
tactile_data_valid: 1
tactile: [2050, 0, 0, 0, 0]
idle_time_us: 1495


**_/rh/palm_extras_**
This topic updates at 84 Hz with data from additional devices plugged into the palm.

Example topic message :
sensors: [1560, 1403, 2836, 1861, 1295, 1555, 3077, 2236, 979, 1270, 2848, 1921, 3150, 1914, 3107, 1817, 2669, 2430, 2195, 2542, 2560, 3767, 645, 1100, 3156, 1636, 26, 27, 28, 29, 30, 31, 0, 0, 0, 1, 0]
motor_data_type:
data: 1
which_motors: 0
which_motor_data_arrived: 345429
which_motor_data_had_errors: 0
motor_data_packet_torque: [-1, -2, -5, 2, 1, -25, 0, -3, -10, -3]
motor_data_packet_misc: [-3, -4, -5, -2, -4, -25, 0, -6, -10, -5]
tactile_data_type: 1
tactile_data_valid: 1
tactile: [2052, 0, 0, 0, 0]
idle_time_us: 267


**_/rh/tactile_**
This topic is published by the driver at 100 Hz with data from tactile sensors.

Example topic message :
tactiles:
-
pac0: 2048
pac1: 2054
pdc: 2533
tac: 2029
tdc: 2556
electrodes: [2622, 3155, 2525, 3062, 2992, 2511, 3083, 137, 2623, 2552, 2928, 3249, 2705, 3037, 3020, 2405, 3049, 948, 2458, 2592, 3276, 3237, 3244, 3119]
-
pac0: 0
pac1: 0
pdc: -9784
tac: 32518
tdc: 0
electrodes: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
-
pac0: 0
pac1: 0
pdc: -9784
tac: 32518
tdc: 0
electrodes: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
-
pac0: 0
pac1: 0
pdc: -9784
tac: 32518
tdc: 0
electrodes: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
-
pac0: 0
pac1: 0
pdc: -9784
tac: 32518
tdc: 0
electrodes: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


**_/rh_trajectory_controller/command_**
This topic can be published to and is the set position for the trajectory controller. It comprises an array of all the joints set positions and is used for commanding the robot.

For example the rqt joint sliders publish to it.

Example topic message :
joint_names: [rh_FFJ1, rh_FFJ2, rh_FFJ3, rh_FFJ4, rh_MFJ1, rh_MFJ2, rh_MFJ3, rh_MFJ4, rh_RFJ1,
rh_RFJ2, rh_RFJ3, rh_RFJ4, rh_LFJ1, rh_LFJ2, rh_LFJ3, rh_LFJ4, rh_LFJ5, rh_THJ1,
rh_THJ2, rh_THJ3, rh_THJ4, rh_THJ5, rh_WRJ1, rh_WRJ2]
points:
-
positions: [0.24434609527920614, 0.8203047484373349, 0.8552113334772214, -0.17453292519943295, 1.0297442586766545, 1.4311699866353502, 1.413716694115407, 0.007182575752410699, 0.9773843811168246, 1.5707963267948966, 1.2566370614359172, -0.12217304763960307, 0.4014257279586958, 1.2566370614359172, 1.5184364492350666, 0.017453292519943295, 0.13962634015954636, 0.12217304763960307, 0.6632251157578453, 0.17453292519943295, 1.117010721276371, -0.7504915783575618, -0.03490658503988659, 0.0]
velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
accelerations: []
effort: []
time_from_start:
secs: 0
nsecs: 5000000


**_/rh_trajectory_controller/state_**
This topic is read-only and update at 50 Hz from the trajectory controller with the positions and velocities of all 24 joints.

Example topic message :
positions: [0.0029928404547430176, 0.0007821521859359137, 0.004102784627362688, -0.001230489872427576, 0.002876479952986344, 0.0006426181816490129, 0.006354919224207833, 0.00213663812281073, 0.003279618063753098, 0.0020929781564538175, 0.0063066586043154516, 0.0038023568140372888, -0.002289758750686488, -1.1040675065743244e-05, 0.008137524637908733, -2.1288137004304986e-05, 0.0009348013388894572, -0.003295237358051928, 0.039981480504079236, -0.0035961821430152696, 0.0032603043080507987, 2.9988784142176428e-05, -0.00029934074598525484, -8.999634459527783e-05]
velocities: [-0.0008510441551395189, -0.0008510441551395189, 0.00016883698712266695, 0.00034715798956923955, -0.00017869100331692196, -0.00017869100331692196, -0.001275520583476054, -0.0004885423191519772, 0.00012555078906251334, 0.00012555078906251334, 0.0028653614401722843, -0.0008023399951605057, 0.0011760287859774613, 0.0011760287859774613, -0.0005423468659163991, -0.00017066612487367117, 0.0003102610817406156, -0.001127052578802167, -0.001465708865391472, -0.00028520412005307133, -0.00029795158858164227, 0.0002596403670543647, -5.819600689424957e-05, -0.0002980347643777659]


/**_sh_rh_*_position_controller/command_**

These topics can be published to and are the set position of each joint in radians. The topics are subscribed to by the driver (/sr_hand_robot node). This topic is used to communicate the set position with the rqt Joint Sliders plugin, when using position control. The Hand can be set to position control using the Change Controllers rqt plugin.

Example of running rostopic info /sh_rh_ffj0_position_controller/command :
Type: std_msgs/Float64
Publishers:

/rqt_gui_py_node_23644 (http://shadow-bravo:38385/)
Subscribers:

/sr_hand_robot (http://shadow-bravo:45091/)

/rostopic_15687_1526406188893 (http://shadow-bravo:36637/)

/record (http://shadow-bravo:35575/)

Example topic message :
data: 0.628318530718


**_/sh_rh_*_position_controller/state_**
These topics are published at 87 Hz by the driver (/sr_hand_robot node). They contain messages of type control_msgs/JointControllerState, which contain the parameters used for the each joints position controller.

Example topic message :
set_point: 1.1113358647
process_value: 1.11095072243
process_value_dot: 0.000426142920695
error: 0.0
time_step: 0.001
command: 0.0
p: -3800.0
i: 0.0d: 0.0
i_clamp: 0.0
antiwindup: False


**_/sh_rh_*_position_controller/max_force_factor_**
The "/sh_rh_*_position_controller/max_force_factor" topic can be published to and scales down the maximum output command of the joints position controller. The output command is interpreted by the driver (/sr_hand_robot node) as PWM if the driver is in PWM mode, or as tendon force if it are in Torque mode.
The maximum force is controlled by the parameter "max_force" that is specified in this yaml file : https://github.com/shadow-robot/sr-config/blob/kinetic-devel/sr_ethercat_hand_config/controls/host/rh/sr_edc_joint_position_controllers_PWM.yaml#L9
"max_force_factor" has a value between [0.0, 1.0] and controls the percentage of the `max_force` that will be effectively considered.
This parameter doesn't exist in the grasp controller.


**_/sh_rh_*_position_controller/pid/parameter_descriptions_**
**_/sh_rh_*_position_controller/pid/parameter_updates_**
These topics are read-only and contain parameters used for tuning the position controllers. They should not be published to directly and are accessed through rqt_reconfigure :


**_/tf_**
**_/tf_static_**
A "tf" is a transform in ROS. These topics store information on the active tfs in the ROS environment and holds their position and orientation in relation their parents. Static tfs are fixed and the dynamic tfs update at 100 Hz.
They can be published to, as well and read from.

For further information on ROS tfs see the ROS wiki : http://wiki.ros.org/tf

Example topic message :
transforms:
-
    header:
     seq: 0
     stamp:
       secs: 1526995980
       nsecs: 100275357
     frame_id: "rh_ffmiddle"
    child_frame_id: "rh_ffdistal"
    transform:
     translation:
       x: 0.0
       y: 0.0
       z: 0.025
     rotation:
       x: 0.641034853577
       y: 0.0
       z: 0.0
       w: 0.767511769617
-
    header:
     seq: 0
     stamp:
       secs: 1526995980
       nsecs: 100275357
     frame_id: "rh_ffproximal"
    child_frame_id: "rh_ffmiddle"
    transform:
     translation:
       x: 0.0
       y: 0.0
       z: 0.045
     rotation:
       x: 0.759399719795
       y: 0.0
       z: 0.0
       w: 0.650624365955

**_/mechanism_statistics_**
This topic is read-only and updates at 1 Hz with the attributes of each joint, for example :
position: 0.715602037549
velocity: 0.0
measured_effort: -11.088
commanded_effort: -10.799974692
is_calibrated: False
violated_limits: False
odometer: 0.0
min_position: 0.715218542352
max_position: 0.715985532746
max_abs_velocity: 0.0363159179688
max_abs_effort: 15.84


**_/ros_ethercat/motors_halted_**
This topic is deprecated - no longer used.
It is a read-only boolean value, updated at 1 Hz, which indicates if the motors have been halted. Generally the value of this is true : http://wiki.ros.org/ethercat_hardware


**_/rosout_**
**_/rosout_agg_**
This is the ROS console log reporting mechanism : http://wiki.ros.org/rosout
The ROS core node, rosout subscribes to the standard /rosout topic, records these messages in a textual log file, and rebroadcasts the messages on /rosout_agg

## Moveit! Topics

In Position control the Moveit topics are used for trajectory planning.
It should not be necessary to interface with these topics, which are described in their documentation here : https://moveit.ros.org/documentation/

These topics provide information about positions, velocities and accelerations of joints whilst executing a trajectory from the current pose to the goal pose.
/rh_trajectory_controller/follow_joint_trajectory/cancel
Used to stop a currently executing trajectory.
/rh_trajectory_controller/follow_joint_trajectory/feedback
/rh_trajectory_controller/follow_joint_trajectory/goal
/rh_trajectory_controller/follow_joint_trajectory/result
/rh_trajectory_controller/follow_joint_trajectory/status

/attached_collision_object
/collision_object
These are used for object collision avoidance if it is active.
/execute_trajectory/cancel
/execute_trajectory/feedback
/execute_trajectory/goal
/execute_trajectory/result
/execute_trajectory/status
Live information regarding the current trajectory execution.
/move_group/cancel
/move_group/display_contacts
/move_group/display_planned_path
/move_group/feedback
/move_group/goal
/move_group/monitored_planning_scene
/move_group/ompl/parameter_descriptions
/move_group/ompl/parameter_updates
/move_group/plan_execution/parameter_descriptions
/move_group/plan_execution/parameter_updates
/move_group/planning_scene_monitor/parameter_descriptions
/move_group/planning_scene_monitor/parameter_updates
/move_group/result
/move_group/sense_for_plan/parameter_descriptions
/move_group/sense_for_plan/parameter_updates
/move_group/status
/move_group/trajectory_execution/parameter_descriptions
/move_group/trajectory_execution/parameter_updates
Information from the move_group node : https://moveit.ros.org/documentation/concepts/
/pickup/cancel
/pickup/feedback
/pickup/goal
/pickup/result
/pickup/status
/place/cancel
/place/feedback
/place/goal
/place/result
/place/status
/planning_scene
/planning_scene_world
/recognized_object_array
/trajectory_execution_event
/filtered
RViz Topics

These topics are used to interface with RViz. Documentation for this can be found here : http://wiki.ros.org/rviz#User_Documentation
/rviz_*/motionplanning_planning_scene_monitor/parameter_descriptions
/rviz_*/motionplanning_planning_scene_monitor/parameter_updates
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update
/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/update_full 
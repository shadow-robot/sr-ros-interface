#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sr_arm_commander import SrArmCommander
from sr_hand_commander import SrHandCommander

rospy.init_node("basic_example", anonymous=True)

# hand = SrHandCommander()
# hand.move_to_joint_value_target({
#     "THJ1": 0, "THJ2": 6, "THJ3": 10, "THJ4": 37, "THJ5": 9,
#     "FFJ0": 21, "FFJ3": 26, "FFJ4": 0,
#     "MFJ0": 18, "MFJ3": 24, "MFJ4": 0,
#     "RFJ0": 30, "RFJ3": 16, "RFJ4": 0,
#     "LFJ0": 30, "LFJ3": 0, "LFJ4": -10, "LFJ5": 10
# })
# rospy.sleep(rospy.Duration(3))
#
# hand_joint_states = hand.get_joints_position()
#
# print("Hand joints positions \n " + str(hand_joint_states) + "\n")

arm = SrArmCommander()

rospy.sleep(rospy.Duration(2))

position_1 = [-0.028412, 0.17665, 0.85672]
print("Moving arm to position\n" + str(position_1) + "\n")
arm.move_to_position_target(position_1)

rospy.sleep(rospy.Duration(3))

position_2 = [0.25527, 0.86682, 0.5426]
print("Moving arm to position\n" + str(position_2) + "\n")
arm.move_to_position_target(position_2)

rospy.sleep(rospy.Duration(3))

print("Arm joints position\n" + str(arm.get_joints_position()) + "\n")

joints_states_1 = {'ra_shoulder_pan_joint': 0.5157461682721474, 'ra_elbow_joint': 0.6876824920327893,
                   'ra_wrist_1_joint': -0.7695210732233582, 'ra_wrist_2_joint': 0.2298871642157314,
                   'ra_shoulder_lift_joint': -0.9569080092786892, 'ra_wrist_3_joint': -0.25991215955733704}
print("Moving arm to joints state\n" + str(joints_states_1) + "\n")

arm.move_to_joint_value_target(joints_states_1)

rospy.sleep(rospy.Duration(3))

joints_states_2 = {'ra_shoulder_pan_joint': 1.9499124556292102, 'ra_elbow_joint': 1.7086485350908838,
                   'ra_wrist_1_joint': -2.4521844853284804, 'ra_wrist_2_joint': -1.7170695649952803,
                   'ra_shoulder_lift_joint': -2.7962420990357364, 'ra_wrist_3_joint': 0.2431812178998669}

print("Moving arm to joints state\n" + str(joints_states_2) + "\n")
arm.move_to_joint_value_target(joints_states_2)

rospy.sleep(rospy.Duration(3))

print("Arm joints position\n" + str(arm.get_joints_position()) + "\n")

joints_states_3 = {'ra_shoulder_pan_joint': 1.6113530596480121, 'ra_elbow_joint': 1.1552231775506083,
                   'ra_wrist_1_joint': -0.2393325455779891, 'ra_wrist_2_joint': 0.4969532212998553,
                   'ra_shoulder_lift_joint': -1.5826889903403423, 'ra_wrist_3_joint': 2.1117520537195738}

print("Running joints trajectory")

joint_trajectory = JointTrajectory()
joint_trajectory.header.stamp = rospy.Time.now()
joint_trajectory.joint_names = list(joints_states_1.keys())
joint_trajectory.points = []
time_from_start = rospy.Duration(5)

for joints_states in [joints_states_1, joints_states_2, joints_states_3]:
    trajectory_point = JointTrajectoryPoint()
    trajectory_point.time_from_start = time_from_start
    time_from_start = time_from_start + rospy.Duration(5)

    trajectory_point.positions = []
    trajectory_point.velocities = []
    trajectory_point.accelerations = []
    trajectory_point.effort = []
    for key in joint_trajectory.joint_names:
        trajectory_point.positions.append(joints_states[key])
        trajectory_point.velocities.append(0.0)
        trajectory_point.accelerations.append(0.0)
        trajectory_point.effort.append(0.0)

    joint_trajectory.points.append(trajectory_point)

arm.run_joint_trajectory(joint_trajectory)

rospy.sleep(rospy.Duration(3))

print("Arm joints position\n" + str(arm.get_joints_position()) + "\n")
print("Arm joints velocities\n" + str(arm.get_joints_velocity()) + "\n")


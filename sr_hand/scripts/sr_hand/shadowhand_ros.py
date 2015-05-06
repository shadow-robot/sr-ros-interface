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
import time
import os
import math
import rospy
import rospkg
import threading
import rosgraph.masterapi
import control_msgs.msg
from sr_robot_msgs.msg import sendupdate, joint, joints_data, JointControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from sr_hand.grasps_parser import GraspParser
from sr_hand.grasps_interpoler import GraspInterpoler
from sr_hand.tactile_receiver import TactileReceiver
# this is only used to detect if the hand is a Gazebo hand, not to actually reconfigure anything
from dynamic_reconfigure.msg import Config


class Joint(object):
    def __init__(self, name="", motor="", j_min=0, j_max=90):
        self.name = name
        self.motor = motor
        self.min = j_min
        self.max = j_max


class ShadowHand_ROS(object):
    """
    This is a python library used to easily access the shadow hand ROS interface.
    """
    def __init__(self):
        """
        Builds the library, creates the communication node in ROS and
        initializes the hand publisher and subscriber to the default
        values of shadowhand_data and sendupdate
        """
        self.allJoints = [Joint("THJ1", "smart_motor_th1", 0, 90),
                          Joint("THJ2", "smart_motor_th2", -40, 40),
                          Joint("THJ3", "smart_motor_th3", -15, 15),
                          Joint("THJ4", "smart_motor_th4", 0, 75),
                          Joint("THJ5", "smart_motor_th5", -60, 60),
                          Joint("FFJ0", "smart_motor_ff2", 0, 180),
                          Joint("FFJ3", "smart_motor_ff3"),
                          Joint("FFJ4", "smart_motor_ff4", -20, 20),
                          Joint("MFJ0", "smart_motor_mf2", 0, 180),
                          Joint("MFJ3", "smart_motor_mf3"),
                          Joint("MFJ4", "smart_motor_mf4", -20, 20),
                          Joint("RFJ0", "smart_motor_rf2", 0, 180),
                          Joint("RFJ3", "smart_motor_rf3"),
                          Joint("RFJ4", "smart_motor_rf4", -20, 20),
                          Joint("LFJ0", "smart_motor_lf2", 0, 180),
                          Joint("LFJ3", "smart_motor_lf3"),
                          Joint("LFJ4", "smart_motor_lf4", -20, 20),
                          Joint("LFJ5", "smart_motor_lf5", 0, 45),
                          Joint("WRJ1", "smart_motor_wr1", -45, 30),
                          Joint("WRJ2", "smart_motor_wr2", -30, 10),
                           ]
        self.handJoints = []
        self.armJoints = [Joint("ShoulderJRotate", "", -45, 60),
                          Joint("ShoulderJSwing", "", 0, 80),
                          Joint("ElbowJSwing", "", 0, 120),
                          Joint("ElbowJRotate", "", -80, 80)
                         ]
        self.lastMsg = joints_data()
        self.lastArmMsg = joints_data()
        self.cyberglove_pub = 0
        self.cyberglove_sub = 0
        self.cybergrasp_pub = 0
        self.cyberglove_sub = 0
        self.isFirstMessage = True
        self.isFirstMessageArm = True
        self.isReady = False
        self.liste = 0
        self.hasarm = 0
        self.dict_pos = {}
        self.dict_tar = {}
        self.dict_arm_pos = {}
        self.dict_arm_tar = {}
        self.dict_ethercat_joints = {}
        self.eth_publishers = {}
        self.eth_subscribers = {}
        #rospy.init_node('python_hand_library')
        self.sendupdate_lock = threading.Lock()

        self.joint_states_lock = threading.Lock()

        #contains the ending for the topic depending on which controllers are loaded
        self.topic_ending = ""


        ##EtherCAT hand
        self.activate_etherCAT_hand()

        ###
        # Grasps
        self.grasp_parser = GraspParser()

        self.rootPath = rospkg.RosPack().get_path('sr_hand')
        self.grasp_parser.parse_tree(self.rootPath+"/scripts/sr_hand/grasps.xml")

        self.grasp_interpoler = 0
        self.pub = rospy.Publisher('srh/sendupdate', sendupdate, queue_size=1, latch=True)
        self.pub_arm = rospy.Publisher('sr_arm/sendupdate', sendupdate, queue_size=1, latch=True)

        self.sub_arm = rospy.Subscriber('sr_arm/shadowhand_data', joints_data,self.callback_arm)
        self.sub = rospy.Subscriber('srh/shadowhand_data', joints_data ,self.callback)

        self.hand_type = self.check_hand_type()

        self.hand_velocity = {}
        self.hand_effort = {}

        if (self.hand_type == "etherCAT") or (self.hand_type == "gazebo"):
            self.joint_states_listener = rospy.Subscriber("joint_states", JointState, self.joint_states_callback)
            # Initialize the command publishers here, to avoid the delay caused when initializing them in the sendupdate
            for jnt in self.allJoints:
                if jnt.name not in self.eth_publishers:
                    topic = "sh_"+ jnt.name.lower() + self.topic_ending+"/command"
                    self.eth_publishers[jnt.name] = rospy.Publisher(topic, Float64, queue_size=1, latch=True)
        self.tactile_receiver = TactileReceiver()

        threading.Thread(None, rospy.spin)

    def create_grasp_interpoler(self, current_step, next_step):
        self.grasp_interpoler = GraspInterpoler(current_step, next_step)

    def callback_ethercat_states(self, data, jointName):
        """
        @param data: The ROS message received which called the callback
        If the message is the first received, initializes the dictionnaries
        Else, it updates the lastMsg
        @param joint: a Joint object that contains the name of the joint that we receive data from
        """
        joint_data = joint(joint_name=jointName, joint_target=math.degrees(float(data.set_point)), joint_position=math.degrees(float(data.process_value)))

        # update the dictionary of joints
        self.dict_ethercat_joints[joint_data.joint_name]=joint_data

        # Build a CAN hand style lastMsg with the latest info in self.dict_ethercat_joints
        self.lastMsg = joints_data()

        for joint_name in self.dict_ethercat_joints.keys() :
            self.lastMsg.joints_list.append(self.dict_ethercat_joints[joint_name])
            #self.lastMsg.joints_list_length++

        #TODO This is not the right way to do it. We should check that messages from all the joints have been
        #  received (but we don't know if we have all the joints, e.g three finger hand)
        self.isReady = True

        # if self.isFirstMessage :
        #     self.init_actual_joints()
        #     for joint in self.lastMsg.joints_list :
        #         self.dict_pos[joint.joint_name]=joint.joint_position
        #         self.dict_tar[joint.joint_name]=joint.joint_target
        #     self.isFirstMessage = False
        #     self.isReady = True

    def callback(self, data):
        """
        @param data: The ROS message received which called the callback
        If the message is the first received, initializes the dictionnaries
        Else, it updates the lastMsg
        """
        self.lastMsg = data;
        if self.isFirstMessage :
            self.init_actual_joints()
            for jnt in self.lastMsg.joints_list :
                self.dict_pos[jnt.joint_name]=jnt.joint_position
                self.dict_tar[jnt.joint_name]=jnt.joint_target
            self.isFirstMessage = False
            self.isReady = True

    def callback_arm(self, data):
        """
        @param data: The ROS message received which called the callback
        If the message is the first received, initializes the dictionnaries
        Else, it updates the lastMsg
        """
        self.lastArmMsg = data
        if self.isFirstMessageArm :
            for jnt in self.lastArmMsg.joints_list :
                self.dict_arm_pos[jnt.joint_name]=jnt.joint_position
                self.dict_arm_tar[jnt.joint_name]=jnt.joint_target

    def init_actual_joints(self):
        """
        Initializes the library with just the fingers actually connected
        """
        for joint_all in self.allJoints :
            for joint_msg in self.lastMsg.joints_list :
                if joint_msg.joint_name == joint_all.name:
                    self.handJoints.append(joint_all)

    def check_hand_type(self):
        """
        @return : true if some hand is detected
        """
        if self.check_gazebo_hand_presence():
            return "gazebo"
        elif self.check_etherCAT_hand_presence():
            return "etherCAT"
        elif self.check_CAN_hand_presence():
            return "CANhand"
        return None

    def check_CAN_hand_presence(self):
        """
        @return : true if the CAN hand is detected
        """
        t = 0.0
        while not self.isReady:
            time.sleep(1.0)
            t = t+1.0
            rospy.loginfo("Waiting for service since "+str(t)+" seconds...")
            if t >= 5.0:
                rospy.logerr("No hand found. Are you sure the ROS hand is running ?")
                return False
        return True

    def set_shadowhand_data_topic(self, topic):
        """
        @param topic: The new topic to be set as the hand publishing topic
        Set the library to listen to a new topic
        """
        print 'Changing subscriber to ' + topic
        self.sub = rospy.Subscriber(topic, joints_data ,self.callback)

    def set_sendupdate_topic(self, topic):
        """
        @param topic: The new topic to be set as the hand subscribing topic
        Set the library to publish to a new topic
        """
        print 'Changing publisher to ' + topic
        self.pub = rospy.Publisher(topic,sendupdate, latch=True)

    def sendupdate_from_dict(self, dicti):
        """
        @param dicti: Dictionnary containing all the targets to send, mapping the name of the joint to the value of its target
        Sends new targets to the hand from a dictionnary
        """
        self.sendupdate_lock.acquire()

        if (self.hand_type == "etherCAT") or (self.hand_type == "gazebo"):
            for join in dicti.keys():

                if join not in self.eth_publishers:
                    topic = "sh_"+ join.lower() + self.topic_ending+"/command"
                    self.eth_publishers[join] = rospy.Publisher(topic, Float64, latch=True)

                msg_to_send = Float64()
                msg_to_send.data = math.radians( float( dicti[join] ) )
                self.eth_publishers[join].publish(msg_to_send)
        elif self.hand_type == "CANhand":
            message = []
            for join in dicti.keys():
                message.append(joint(joint_name=join, joint_target=dicti[join]))
            self.pub.publish(sendupdate(len(message), message))

        self.sendupdate_lock.release()

    def sendupdate(self, jointName, angle=0):
        """
        @param jointName: Name of the joint to update
        @param angle: Target of the joint, 0 if not set
        Sends a new target for the specified joint
        """
        self.sendupdate_lock.acquire()

        if (self.hand_type == "etherCAT") or (self.hand_type == "gazebo"):
            if jointName not in self.eth_publishers:
                topic = "sh_"+ jointName.lower() + self.topic_ending + "/command"
                self.eth_publishers[jointName] = rospy.Publisher(topic, Float64, latch=True)

            msg_to_send = Float64()
            msg_to_send.data = math.radians( float( angle ) )
            self.eth_publishers[jointName].publish(msg_to_send)
        elif self.hand_type == "CANhand":
            message = [joint(joint_name=jointName, joint_target=angle)]
            self.pub.publish(sendupdate(len(message), message))

        self.sendupdate_lock.release()

    def sendupdate_arm_from_dict(self, dicti):
        """
        @param dicti: Dictionnary containing all the targets to send, mapping the name of the joint to the value of its target
        Sends new targets to the hand from a dictionnary
        """
        message = []
        for join in dicti.keys():
            message.append(joint(joint_name=join, joint_target=dicti[join]))
        self.pub_arm.publish(sendupdate(len(message), message))

    def sendupdate_arm(self, jointName, angle=0):
        """
        @param jointName: Name of the joint to update
        @param angle: Target of the joint, 0 if not set
        Sends a new target for the specified joint
        """
        message = [joint(joint_name=jointName, joint_target=angle)]
        self.pub_arm.publish(sendupdate(len(message), message))

    def valueof(self, jointName):
        """
        @param jointName: Name of the joint to read the value
        @return: 'NaN' if the value is not correct, the actual position of the joint else
        """
        for jnt in self.lastMsg.joints_list:
            if jnt.joint_name == jointName:
                return float(jnt.joint_position)
        for jnt in self.lastArmMsg.joints_list:
            if jnt.joint_name == jointName:
                return float(jnt.joint_position)
        return 'NaN'

    def has_arm(self):
        """
        @return : True if an arm is detected on the roscore
        """
        if not self.hasarm == 0:
            return self.hasarm
        self.hasarm = False
        if self.liste == 0:
            master = rosgraph.masterapi.Master('/rostopic')
            self.liste = master.getPublishedTopics('/')
        for topic_typ in self.liste :
            for topic in topic_typ:
                if 'sr_arm/shadowhand_data' in topic :
                    self.hasarm = True
        return self.hasarm

    def record_step_to_file(self, filename, grasp_as_xml):
        """
        @param filename: name (or path) of the file to save to
        @param grasp_as_xml: xml-formatted grasp
        Write the grasp at the end of the file, creates the file if does not exist
        """
        if os.path.exists(filename) :
            obj = open(filename, 'r')
            text = obj.readlines()
            obj.close()
            objWrite = open(filename,'w')
            for index in range(0,len(text)-1):
                objWrite.write(text[index])
            objWrite.write(grasp_as_xml)
            objWrite.write('</root>')
            objWrite.close()
        else :
            objWrite = open(filename,'w')
            objWrite.write('<root>\n')
            objWrite.write(grasp_as_xml)
            objWrite.write('</root>')
            objWrite.close()

    def save_hand_position_to_file(self, filename):
        objFile=open(filename,'w')
        for key, value in self.dict_pos:
            objFile.write(key + ' ' + value + '\n')
        objFile.close()

    def read_all_current_positions(self):
        """
        @return: dictionnary mapping joint names to actual positions
        Read all the positions in the lastMsg
        """
        if not self.isReady:
            return
        for jnt in self.lastMsg.joints_list:
            self.dict_pos[jnt.joint_name] = jnt.joint_position
        return self.dict_pos

    def read_all_current_targets(self):
        """
        @return: dictionnary mapping joint names to current targets
        Read all the targets in the lastMsg
        """
        for jnt in self.lastMsg.joints_list:
            self.dict_tar[jnt.joint_name] = jnt.joint_target
        return self.dict_tar

    def read_all_current_velocities(self):
        """
        @return: dictionary mapping joint names to current velocities
        """
        with self.joint_states_lock:
            return self.hand_velocity

    def read_all_current_efforts(self):
        """
        @return: dictionary mapping joint names to current efforts
        """
        with self.joint_states_lock:
            return self.hand_effort

    def read_all_current_arm_positions(self):
        """
        @return: dictionnary mapping joint names to actual positions
        Read all the positions in the lastMsg
        """
        for jnt in self.lastArmMsg.joints_list:
            self.dict_arm_pos[jnt.joint_name] = jnt.joint_position
        return self.dict_arm_pos

    def read_all_current_arm_targets(self):
        """
        @return: dictionnary mapping joint names to actual targets
        Read all the targets in the lastMsg
        """
        for jnt in self.lastArmMsg.joints_list:
            self.dict_arm_tar[jnt.joint_name] = jnt.joint_target
        return self.dict_arm_tar

    def resend_targets(self):
        """
        Resend the targets read in the lastMsg to the hand
        """
        for key, value in self.dict_tar.items():
            self.sendupdate(jointName=key, angle=value)

    def callVisualisationService(self, callList=0, reset = 0):
        """
        @param callList: dictionnary mapping joint names to information that should be displayed
        @param reset: flag used to tell if the parameters should be replaced by the new ones or just added to the previous ones
        Calls a ROS service to display various information in Rviz
        """
        if reset == 0:
            print 'no reset'

        if reset == 1:
            print 'reset'

    def check_etherCAT_hand_presence(self):
        """
        Only used to check if a real etherCAT hand is detected in the system
        check if something is being published to this topic, otherwise
        return false
        Bear in mind that the gazebo hand also publishes the joint_states topic,
        so we need to check for the gazebo hand first
        """

        try:
            rospy.wait_for_message("joint_states", JointState, timeout = 0.2)
        except rospy.ROSException:
            rospy.logwarn("no message received from joint_states")
            return False

        return True

    def check_gazebo_hand_presence(self):
        """
        Only used to check if a Gazebo simulated (etherCAT protocol) hand is detected in the system
        check if something is being published to this topic, otherwise
        return false
        """

        try:
            rospy.wait_for_message("gazebo/parameter_updates", Config, timeout = 0.2)
        except rospy.ROSException:
            return False

        return True

    def activate_etherCAT_hand(self):
        """
        At the moment we just try to use the mixed position velocity controllers
        """
        success = True
        for joint_all in self.allJoints :
            self.topic_ending = "_position_controller"
            topic = "sh_"+ joint_all.name.lower() + self.topic_ending + "/state"
            success = True
            try:
                rospy.wait_for_message(topic, control_msgs.msg.JointControllerState, timeout = 0.2)
            except:
                try:
                    self.topic_ending = "_mixed_position_velocity_controller"
                    topic = "sh_"+ joint_all.name.lower() + self.topic_ending + "/state"
                    rospy.wait_for_message(topic, JointControllerState, timeout = 0.2)
                except rospy.ROSException:
                    success = False

            if success:
                if self.topic_ending == "_mixed_position_velocity_controller":
                    self.eth_subscribers[joint_all.name] = rospy.Subscriber(topic, JointControllerState, self.callback_ethercat_states, joint_all.name)
                else:
                    self.eth_subscribers[joint_all.name] = rospy.Subscriber(topic,
                                                                            control_msgs.msg.JointControllerState,
                                                                            self.callback_ethercat_states,
                                                                            joint_all.name)

        if len(self.eth_subscribers) > 0:
            return True

        return False

    def joint_states_callback(self, joint_state):
        """
        The callback function for the topic joint_states.
        It will store the received joint velocity and effort information in two dictionaries
        Velocity will be converted to degrees/s.
        Effort units are kept as they are (currently ADC units, as no calibration is performed on the strain gauges)

        @param joint_state: the message containing the joints data.
        """
        with self.joint_states_lock:
            self.hand_velocity = {n:math.degrees(v) for n,v in zip(joint_state.name, joint_state.velocity)}
            self.hand_effort = {n:e for n,e in zip(joint_state.name, joint_state.effort)}

            for finger in ['FF', 'MF', 'RF', 'LF']:
                for dic in [self.hand_velocity, self.hand_effort]:
                    if (finger + 'J1') in dic and (finger + 'J2') in dic:
                        dic[finger + 'J0'] = dic[finger + 'J1'] + dic[finger + 'J2']
                        del dic[finger + 'J1']
                        del dic[finger + 'J2']

    def get_tactile_type(self):
        return self.tactile_receiver.get_tactile_type()

    def get_tactile_state(self):
        return self.tactile_receiver.get_tactile_state()

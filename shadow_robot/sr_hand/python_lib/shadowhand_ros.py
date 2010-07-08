#!/usr/bin/env python
import roslib; roslib.load_manifest('sr_control_gui')
import time
import os
import math
import rospy
import subprocess
import threading
import rosgraph.masterapi
from sr_hand.msg import sendupdate, joint, joints_data
from sensor_msgs.msg import *

class Joint():
    def __init__(self, name="", motor="", min=0, max=90):
        self.name = name
        self.motor = motor
        self.min = min
        self.max = max

class ShadowHand_ROS():
    """
    This is a python library used to easily access the shadow hand ROS interface. 
    """
    def __init__(self):
        """
        Builds the library, creates the communication node in ROS and
        initializes the hand publisher and subscriber to the default 
        values of shadowhand_data and sendupdate 
        """
        print "Creating good library"
        self.handJoints = [Joint("THJ1", "smart_motor_th1"), 
                           Joint("THJ2", "smart_motor_th2", -30, 30), 
                           Joint("THJ3", "smart_motor_th3",-15, 15),
                           Joint("THJ4", "smart_motor_th4",0, 75),
                           Joint("THJ5", "smart_motor_th5",-60, 60),
                           Joint("FFJ0", "smart_motor_ff2", 0, 180),
                           Joint("FFJ3", "smart_motor_ff3"),
                           Joint("FFJ4", "smart_motor_ff4", -25, 25),
                           Joint("MFJ0", "smart_motor_mf2", 0, 180),
                           Joint("MFJ3", "smart_motor_mf3"),
                           Joint("MFJ4", "smart_motor_mf4", -25, 25),
                           Joint("RFJ0", "smart_motor_rf2", 0, 180),
                           Joint("RFJ3", "smart_motor_rf3"),
                           Joint("RFJ4", "smart_motor_rf4", -25,25),
                           Joint("LFJ0", "smart_motor_lf2", 0, 180),
                           Joint("LFJ3", "smart_motor_lf3"),
                           Joint("LFJ4", "smart_motor_lf4", -25, 25), 
                           Joint("LFJ5", "smart_motor_lf5", 0, 45),
                           Joint("WRJ1", "smart_motor_wr1", -30, 40), 
                           Joint("WRJ2", "smart_motor_wr2", -30, 10),
                           ]
        self.armJoints = [Joint("trunk_rotation", "", -45, 90),
                          Joint("shoulder_rotation", "", 0, 90),
                          Joint("elbow_abduction", "", 0,120),
                          Joint("forearm_rotation", "", -90,90)
                         ]
        self.lastMsg = 0;
        self.lastArmMsg = 0;
        self.cyberglove_pub = 0
        self.cyberglove_sub = 0
        self.cybergrasp_pub = 0
        self.cyberglove_sub = 0
        self.sub = rospy.Subscriber('srh/shadowhand_data', joints_data ,self.callback)
        self.pub = rospy.Publisher('srh/sendupdate',sendupdate)
        self.sub_arm = rospy.Subscriber('sr_arm/shadowhand_data', joints_data,self.callback_arm)
        self.pub_arm = rospy.Publisher('sr_arm/sendupdate',sendupdate)
        self.isFirstMessage = True
        self.isFirstMessageArm = True
        self.isReady = False
        self.toRecord=0
        self.liste = 0
        self.hasarm = 0
        self.jointToRecord = 'None'
        self.dict_pos = {}
        self.dict_tar = {}
        self.dict_arm_pos = {}
        self.dict_arm_tar = {}
        self.dict_record_pos = {}
        self.dict_record_tar={}
        self.dict_record_time={}
        rospy.init_node('python_hand_library')
        threading.Thread(None, rospy.spin)

    def callback(self, data):
        """
        @param data: The ROS message received which called the callback
        If the message is the first received, initializes the dictionnaries
        Else, it updates the lastMsg    
        """
        self.lastMsg = data;        
        if self.isFirstMessage : 
            for joint in self.lastMsg.joints_list : 
                self.dict_pos[joint.joint_name]=joint.joint_position
                self.dict_tar[joint.joint_name]=joint.joint_target
            self.dict_record_pos[joint.joint_name]=[]
            self.dict_record_tar[joint.joint_name]=[]
            self.isFirstMessage = False
            self.isReady = True
        if self.toRecord > 0 : 
            self.dict_record_pos[self.jointToRecord].append(self.dict_pos[self.jointToRecord])
            self.dict_record_tar[self.jointToRecord].append(self.dict_tar)
            self.dict_record_time[self.jointToRecord].append(rospy.Time.now())
            toRecord = toRecord - 1

    def callback_arm(self, data):
        """
        @param data: The ROS message received which called the callback
        If the message is the first received, initializes the dictionnaries
        Else, it updates the lastMsg    
        """
        self.lastArmMsg = data    
        if self.isFirstMessageArm :
            for joint in self.lastArmMsg.joints_list : 
                self.dict_arm_pos[joint.joint_name]=joint.joint_position
                self.dict_arm_tar[joint.joint_name]=joint.joint_target
        

    def __del__(self):
        print('Library deleted')

    def close_connections(self):
        print('Connection closed')
    
    def setShadowhand_data_topic(self, topic):
        """
        @param topic: The new topic to be set as the hand publishing topic
        Set the library to listen to a new topic
        """
        print 'Changing subscriber to ' + topic 
        self.sub = rospy.Subscriber(topic, joints_data ,self.callback)
    
    def setSendUpdate_topic(self, topic):
        """
        @param topic: The new topic to be set as the hand subscribing topic
        Set the library to publish to a new topic
        """
        print 'Changing publisher to ' + topic 
        self.pub = rospy.Publisher(topic,sendupdate)

    def sendupdate_from_dict(self, dicti):
        """
        @param dicti: Dictionnary containing all the targets to send, mapping the name of the joint to the value of its target
        Sends new targets to the hand from a dictionnary
        """
        #print(dicti)
        message = []
        for join in dicti.keys():
            message.append(joint(joint_name=join, joint_target=dicti[join]))
        self.pub.publish(sendupdate(len(message), message))

    def sendupdate(self, jointName, angle=0):
        """
        @param jointName: Name of the joint to update
        @param angle: Target of the joint, 0 if not set
        Sends a new target for the specified joint
        """
        message = [joint(joint_name=jointName, joint_target=angle)]
        self.pub.publish(sendupdate(len(message), message))

    def sendupdate_arm_from_dict(self, dicti):
        """
        @param dicti: Dictionnary containing all the targets to send, mapping the name of the joint to the value of its target
        Sends new targets to the hand from a dictionnary
        """
        #print(dicti)
        #print(dicti)
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
        self.pub_arm.publish(sendupdate_arm(len(message), message))

    def valueof(self, jointName):
        """
        @param jointName: Name of the joint to read the value
        @return: 'NaN' if the value is not correct, the actual position of the joint else
        """
        for joint in self.lastMsg.joints_list:
            if joint.joint_name == jointName:
                return float(joint.joint_position)
        for joint in self.lastArmMsg.joints_list:
            if joint.joint_name == jointName:
                return float(joint.joint_position)
        return 'NaN'        

    def has_arm(self):
        """
        @return : True if an arm is detected on the roscore
        """
        if not self.hasarm == 0:
            return self.hasarm
        self.hasarm = False
        if self.liste == 0:
            #process = subprocess.Popen("rostopic list -p".split(), stdout=subprocess.PIPE)
            #self.liste =  process.communicate()[0]
            #self.liste = self.liste.split('\n')
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
        for joint in self.lastMsg.joints_list:
            self.dict_pos[joint.joint_name] = joint.joint_position
        return self.dict_pos

    def read_all_current_targets(self):
        """
        @return: dictionnary mapping joint names to current targets
        Read all the targets in the lastMsg
        """
        for joint in self.lastMsg.joints_list:
            self.dict_tar[joint.joint_name] = joint.joint_position
        return self.dict_tar

    def read_all_current_arm_positions(self):
        """
        @return: dictionnary mapping joint names to actual positions
        Read all the positions in the lastMsg
        """
        for joint in self.lastArmMsg.joints_list:
            self.dict_arm_pos[joint.joint_name] = joint.joint_position
        return self.dict_arm_pos

    def read_all_current_arm_targets(self):
        """
        @return: dictionnary mapping joint names to actual targets
        Read all the targets in the lastMsg
        """
        for joint in self.lastArmMsg.joints_list:
            self.dict_arm_tar[joint.joint_name] = joint.joint_target
        return self.dict_arm_tar

    def resend_targets(self):
        """
        Resend the targets read in the lastMsg to the hand
        """
        for key, value in self.dict_tar.items():
            self.sendupdate(jointName=key, angle=value)

    def replay_hand_position_from_file(self, filename):
        """
        Depreciated, should not be used
        """
        dict_temp={}
        objFile=open(filename,'r')
        result = objFile.readlines()
        for line in result : 
            split=line.split()
            name=split[0]
            angle=split[1]
            dict_temp[name]=angle
        objFile.close()
        self.sendupdate_from_dict(dict_temp)

    def readRecordedData(self, jointName, recordingTimeInSeconds):
        """
        Depreciated, should not be used
        """
        ret = []
        self.fileIsComplete(recordingTimeInSeconds*100)
        """ Getting through only when ready"""
        positions = dict_record_pos[jointName]
        targets = dict_record_tar[jointName]
        times = dict_record_time[jointName]            
        for index in range(0, len(targets)) :
            ret.append([int(times[i]), float(positions[i]), float(targets[i])])
        minTime = ret[0][0]
        for index in range(0, len(ret)) : 
            ret[index][0] = float((ret[index][0]-minTime))/1000000000.0

        return ret


    def tune(self):
        print('TODO')

    def initializeJointsPosition(self):
        print('TODO')

    def saveContrlrSettings(self):
        print('TODO')

    def stepFunction(self):
        self.sendupdate(joint.name, joint.min + 0.6*(joint.max - joint.min))

    def smoothSinFunction(self, joint):
        numberOfSteps = 500
        for index in range(numberOfSteps):
            theta = (float(index)/float(numberOfSteps)) * math.pi
            mysinvalue = math.sin(theta)
            self.sendupdate(joint.name, joint.min + mysinvalue*(joint.max - joint.min))
            time.sleep(0.01)
        return 0


    def staircaseFunction(self):
        for percentage in [0, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80,     0.90, 1.00, 0.90 , 0.80, 0.70, 0.60, 0.50, 0.40, 0.30, 0.20, 0.10, 0]:
            self.sendupdate(joint.name, joint.min + percentage*(joint.max - joint.min))
            time.sleep(0.25)
        return 0

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


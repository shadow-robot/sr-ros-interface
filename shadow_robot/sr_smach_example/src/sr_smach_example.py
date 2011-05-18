#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_smach_example')
import rospy
import smach
import smach_ros
from sr_robot_msgs.msg import sendupdate, joint, joints_data
import time
import threading

class Position(object):
    def __init__(self):
        self.safe_pos_msg = []
        self.init_safe_pos_msg()

        self.first_pos_msg = []
        self.init_first_pos_msg()

        self.hand_publisher = rospy.Publisher('srh/sendupdate', sendupdate)
        self.arm_publisher = rospy.Publisher('sr_arm/sendupdate', sendupdate)

    def init_safe_pos_msg(self):
        pos = {'FFJ0': 160, 'FFJ3': 80, 'FFJ4': 0,
               'MFJ0': 160, 'MFJ3': 80, 'MFJ4': 0,
               'RFJ0': 160, 'RFJ3': 80, 'RFJ4': 0,
               'LFJ0': 160, 'LFJ3': 80, 'LFJ4': 0, 'LFJ5': 0,
               'WRJ1': 0, 'WRJ2': 0,
               'THJ1': 45, 'THJ2': 20, 'THJ3': 0, 'THJ4': 50, 'THJ5': 5,

               'ShoulderJRotate': 0,
               'ShoulderJSwing': 0,
               'ElbowJSwing': 100,
               'ElbowJRotate': 0
               }

        for joint_name in pos.keys():
            self.safe_pos_msg.append(joint(joint_name=joint_name,
                                           joint_target=pos[joint_name]))

    def init_first_pos_msg(self):
        pos = {'FFJ0': 0, 'FFJ3': 0, 'FFJ4': 0,
               'MFJ0': 160, 'MFJ3': 80, 'MFJ4': 0,
               'RFJ0': 160, 'RFJ3': 80, 'RFJ4': 0,
               'LFJ0': 160, 'LFJ3': 80, 'LFJ4': 0, 'LFJ5': 0,
               'WRJ1': 0, 'WRJ2': 0,
               'THJ1': 45, 'THJ2': 20, 'THJ3': 0, 'THJ4': 50, 'THJ5': 5,

               'ShoulderJRotate': -45,
               'ShoulderJSwing': 10,
               'ElbowJSwing': 120,
               'ElbowJRotate': -80
               }

        for joint_name in pos.keys():
            self.first_pos_msg.append(joint(joint_name=joint_name,
                                            joint_target=pos[joint_name]))

class SafePosition(smach.State):
    """
    The robot goes into a safe position
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failed'])

        self.pos = Position()

    def execute(self, userdata):
        rospy.loginfo('Going to a Safe position')

        for i in range(0,10):
            self.pos.hand_publisher.publish(sendupdate(len(self.pos.safe_pos_msg),
                                                       self.pos.safe_pos_msg))
            self.pos.arm_publisher.publish(sendupdate(len(self.pos.safe_pos_msg),
                                                      self.pos.safe_pos_msg))

            time.sleep(.1)

        time.sleep(1)
        return 'success'

class FirstMove(smach.State):
    """
    The robot does its first move.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failed'])
        self.pos = Position()

    def execute(self, userdata):
        rospy.loginfo('Going to the first position')

        for i in range(0,10):
            self.pos.hand_publisher.publish(sendupdate(len(self.pos.first_pos_msg),
                                                       self.pos.first_pos_msg))
            self.pos.arm_publisher.publish(sendupdate(len(self.pos.first_pos_msg),
                                                      self.pos.first_pos_msg))
            time.sleep(.1)

        time.sleep(1)

        return 'success'

class ComeCloser(smach.State):
    """
    Beckons with the first finger
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failed'])
        self.pos = Position()

    def execute(self, userdata):
        rospy.loginfo('Come Closer')

        for i in range(0,3):
            finger_opened = sendupdate(1, [joint(joint_name="FFJ0", joint_target=0)])
            finger_closed = sendupdate(1, [joint(joint_name="FFJ0", joint_target=160)])
            self.pos.hand_publisher.publish(finger_closed)
            time.sleep(1)
            self.pos.hand_publisher.publish(finger_opened)
            time.sleep(1)

        return 'success'

class WaitForRobot(smach.State):
    """
    Waits for the robot to reach the given position: checks if the positions
    are close enough to the targets.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failed'])

        self.goal_mutex = threading.Lock()
        self.goal_reached = False

        self.hand_subscriber = rospy.Subscriber('srh/shadowhand_data', joints_data, self.callback)
        self.arm_subscriber = rospy.Subscriber('sr_arm/shadowhand_data', joints_data, self.callback)

    def callback(self, data):
        for joint_data in data.joints_list:
            error = (joint_data.joint_position - joint_data.joint_target)
            error *= error

            self.goal_mutex.acquire()
            if error < 4.0:
                self.goal_reached = True
            else:
                self.goal_reached = False
            self.goal_mutex.release()

    def execute(self, userdata):
        rospy.loginfo('Come Closer')

        #wait for a maximum of 30 seconds for the robot to reach its targets
        for i in range(0, 300):
            self.goal_mutex.acquire()
            if self.goal_reached:
                self.goal_mutex.release()
                return 'success'
            self.goal_mutex.release()
            time.sleep(.1)

        return 'failed'


def main():
    rospy.init_node('sr_smach_example')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['success', 'failed'])

    # Create and start the introspection server (to be able to display the state machine using
    #  rosrun smach_viewer smach_viewer.py)
    sis = smach_ros.IntrospectionServer('sr_smach_example', sm, '/sr_smach_example')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('GoToStartPos', SafePosition(),
                               transitions={'success':'WaitForRobot_1'})

        smach.StateMachine.add('WaitForRobot_1', WaitForRobot(),
                               transitions={'success':'FirstMove', 'failed':'GoToStopPos'})

        smach.StateMachine.add('FirstMove', FirstMove(),
                               transitions={'success':'WaitForRobot_2'})

        smach.StateMachine.add('WaitForRobot_2', WaitForRobot(),
                               transitions={'success':'ComeCloser', 'failed':'GoToStopPos'})

        smach.StateMachine.add('ComeCloser', ComeCloser(),
                               transitions={'success':'GoToStopPos'})
        smach.StateMachine.add('GoToStopPos', SafePosition())

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

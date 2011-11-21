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

import roslib; roslib.load_manifest('sr_smach_example')
import rospy
import smach
import smach_ros
from sr_robot_msgs.msg import sendupdate, joint, joints_data
import time
import threading


MAX_ITER_COME_CLOSER = 4
MAX_ITER_BIG_MUSCLES = 4
MAX_SQUARE_ERROR = 10


class Position(object):
    def __init__(self):
        self.safe_pos_msg = []
        self.init_safe_pos_msg()

        self.first_pos_msg = []
        self.init_first_pos_msg()

        self.big_muscles_start = []
        self.init_big_muscles_start()

        self.big_muscles_end = []
        self.init_big_muscles_end()

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
    def init_big_muscles_start(self):
        pos = {'FFJ0': 0, 'FFJ3': 0, 'FFJ4': 0,
               'MFJ0': 0, 'MFJ3': 0, 'MFJ4': 0,
               'RFJ0': 0, 'RFJ3': 0, 'RFJ4': 0,
               'LFJ0': 0, 'LFJ3': 0, 'LFJ4': 0, 'LFJ5': 0,
               'WRJ1': 0, 'WRJ2': 0,
               'THJ1': 45, 'THJ2': 20, 'THJ3': 0, 'THJ4': 50, 'THJ5': 5,

               'ShoulderJRotate': 0,
               'ShoulderJSwing': 20,
               'ElbowJSwing': 75,
               'ElbowJRotate': -80
               }

        for joint_name in pos.keys():
            self.big_muscles_start.append(joint(joint_name=joint_name,
                                                joint_target=pos[joint_name]))
    def init_big_muscles_end(self):
        pos = {'FFJ0': 160, 'FFJ3': 80, 'FFJ4': 0,
               'MFJ0': 160, 'MFJ3': 80, 'MFJ4': 0,
               'RFJ0': 160, 'RFJ3': 80, 'RFJ4': 0,
               'LFJ0': 160, 'LFJ3': 80, 'LFJ4': 0, 'LFJ5': 0,
               'WRJ1': 30, 'WRJ2': 0,
               'THJ1': 45, 'THJ2': 20, 'THJ3': 0, 'THJ4': 50, 'THJ5': 5,

               'ShoulderJRotate': -30,
               'ShoulderJSwing': 20,
               'ElbowJSwing': 120,
               'ElbowJRotate': -80
               }

        for joint_name in pos.keys():
            self.big_muscles_end.append(joint(joint_name=joint_name,
                                              joint_target=pos[joint_name]))


class SafePosition(smach.State):
    """
    The robot goes into a safe position
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failed'],
                             output_keys=['index_counter', 'max_counter', 'last_targets'])
        self.pos = Position()
        self.targets = sendupdate(len(self.pos.safe_pos_msg),
                                  self.pos.safe_pos_msg)

    def execute(self, userdata):
        rospy.loginfo('Going to a Safe position')

        self.pos.hand_publisher.publish(self.targets)
        self.pos.arm_publisher.publish(self.targets)

        userdata.last_targets = self.targets
        userdata.index_counter = 0
        userdata.max_counter = 0
        return 'success'

class FirstMove(smach.State):
    """
    The robot does its first move.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failed'],
                             output_keys=['index_counter', 'max_counter', 'last_targets'])
        self.pos = Position()
        self.targets = sendupdate(len(self.pos.first_pos_msg),
                                  self.pos.first_pos_msg)

    def execute(self, userdata):
        rospy.loginfo('Going to the first position')

        self.pos.hand_publisher.publish(self.targets)
        self.pos.arm_publisher.publish(self.targets)

        userdata.last_targets = self.targets
        userdata.index_counter = 0
        userdata.max_counter = 0
        return 'success'

class ComeCloser(smach.State):
    """
    Beckons with the first finger
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['success','failed'],
                             output_keys=['index_counter', 'max_counter', 'last_targets'])
        self.pos = Position()
        self.iteration = 0
        self.opened = True

        self.finger_opened = sendupdate(1, [joint(joint_name="FFJ0", joint_target=0)])
        self.finger_closed = sendupdate(1, [joint(joint_name="FFJ0", joint_target=160)])

    def execute(self, userdata):
        rospy.loginfo('Come Closer')

        if self.opened:
            userdata.last_targets = self.finger_closed
            self.pos.hand_publisher.publish(self.finger_closed)
            self.opened = False
        else:
            userdata.last_targets = self.finger_opened
            self.pos.hand_publisher.publish(self.finger_opened)
            self.opened = True

        self.iteration += 1
        userdata.index_counter = self.iteration
        userdata.max_counter = MAX_ITER_COME_CLOSER
        return 'success'


class BigMuscles(smach.State):
    """
    Flex the arm to show its big muscles.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'failed'],
                             output_keys=['index_counter', 'max_counter', 'last_targets'])
        self.pos = Position()
        self.iteration = 0
        self.big_muscles_start = False

        self.target_1 = sendupdate(len(self.pos.big_muscles_end),
                                   self.pos.big_muscles_end)
        self.target_2 = sendupdate(len(self.pos.big_muscles_start),
                                   self.pos.big_muscles_start)

    def execute(self, userdata):
        rospy.loginfo('Big Muscles')

        if self.big_muscles_start:
            userdata.last_targets = self.target_1
            self.pos.hand_publisher.publish(self.target_1)
            self.pos.arm_publisher.publish(self.target_1)
            self.big_muscles_start = False
        else:
            userdata.last_targets = self.target_2
            self.pos.hand_publisher.publish(self.target_2)
            self.pos.arm_publisher.publish(self.target_2)
            self.big_muscles_start = True

        self.iteration += 1

        userdata.index_counter = self.iteration
        userdata.max_counter = MAX_ITER_BIG_MUSCLES
        return 'success'

class WaitForRobot(smach.State):
    """
    Waits for the robot to reach the given position: checks if the positions
    are close enough to the targets.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'in_progress', 'failed'],
                             input_keys=['index_counter','max_counter', 'last_targets'])

        self.goal_mutex = threading.Lock()
        self.goal_hand_reached = False
        self.goal_arm_reached = False

        self.pos = Position()

        self.hand_subscriber = rospy.Subscriber('srh/shadowhand_data', joints_data, self.callback_hand)
        self.arm_subscriber = rospy.Subscriber('sr_arm/shadowhand_data', joints_data, self.callback_arm)

    def callback_hand(self, data):
        self.goal_mutex.acquire()
        ok = True
        for joint_data in data.joints_list:
            error = (joint_data.joint_position - joint_data.joint_target)
            error *= error

            if error > MAX_SQUARE_ERROR:
                ok = False
                break

        self.goal_hand_reached = True
        self.goal_mutex.release()

    def callback_arm(self, data):
        self.goal_mutex.acquire()
        ok = True
        for joint_data in data.joints_list:
            error = (joint_data.joint_position - joint_data.joint_target)
            error *= error

            if error > MAX_SQUARE_ERROR:
                ok = False
                break

        self.goal_arm_reached = ok
        self.goal_mutex.release()

    def execute(self, userdata):
        rospy.loginfo('Come Closer')

        #wait for a maximum of 30 seconds for the robot to reach its targets
        for i in range(0, 300):
            self.goal_mutex.acquire()
            if self.goal_hand_reached and self.goal_arm_reached:
                self.goal_mutex.release()

                #dummy sleep as we're using the dummy hand
                time.sleep(1)

                if userdata.index_counter < userdata.max_counter:
                    return 'in_progress'
                else:
                    return 'success'
            self.goal_mutex.release()

            #continue publishing the targets until we reach them
            self.pos.hand_publisher.publish(userdata.last_targets)
            self.pos.arm_publisher.publish(userdata.last_targets)

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
                               transitions={'success':'FirstMove', 'in_progress':'GoToStartPos', 'failed':'GoToStopPos'},
                               remapping={'index_counter':'index_counter',
                                          'max_counter':'max_counter',
                                          'last_targets':'last_targets'})

        smach.StateMachine.add('FirstMove', FirstMove(),
                               transitions={'success':'WaitForRobot_2'})

        smach.StateMachine.add('WaitForRobot_2', WaitForRobot(),
                               transitions={'success':'ComeCloser',
                                            'in_progress':'FirstMove',
                                            'failed':'GoToStopPos'},
                               remapping={'index_counter':'index_counter',
                                          'max_counter':'max_counter',
                                          'last_targets':'last_targets'})

        smach.StateMachine.add('ComeCloser', ComeCloser(),
                               transitions={'success':'WaitForRobot_3', 'failed':'GoToStopPos'})

        smach.StateMachine.add('WaitForRobot_3', WaitForRobot(),
                               transitions={'success':'BigMuscles',
                                            'in_progress':'ComeCloser',
                                            'failed':'GoToStopPos'},
                               remapping={'index_counter':'index_counter',
                                          'max_counter':'max_counter',
                                          'last_targets':'last_targets'})

        smach.StateMachine.add('BigMuscles', BigMuscles(),
                               transitions={'success':'WaitForRobot_4', 'failed':'GoToStopPos', })

        smach.StateMachine.add('WaitForRobot_4', WaitForRobot(),
                               transitions={'success':'GoToStopPos',
                                            'in_progress':'BigMuscles',
                                            'failed':'GoToStopPos'},
                               remapping={'index_counter':'index_counter',
                                          'max_counter':'max_counter',
                                          'last_targets':'last_targets'})

        smach.StateMachine.add('GoToStopPos', SafePosition())

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()

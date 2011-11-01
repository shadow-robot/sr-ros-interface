#!/usr/bin/env python

import roslib; roslib.load_manifest('sr_object_manipulation_smach')
import rospy
import smach
import smach_ros

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome1'



# define state Bas
class Bas(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAS')
        return 'outcome3'




def main():
    rospy.init_node('smach_example_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome5'])
    sis = smach_ros.IntrospectionServer('smach_example_state_machine',
                                        sm_top,
                                        '/smach_example_state_machine')
    sis.start()
    sis2=None

    # Open the container
    with sm_top:

        smach.StateMachine.add('BAS', Bas(),
                               transitions={'outcome3':'SUB'})

        # Create the sub SMACH state machine
        sm_sub = smach.StateMachine(outcomes=['outcome4'])
        sis2 = smach_ros.IntrospectionServer('SUB',
                                             sm_sub,
                                             '/SUB')
        sis2.start()
        # Open the container
        with sm_sub:
            # Add states to the container
            smach.StateMachine.add('FOO', Foo(),
                                   transitions={'outcome1':'BAR',
                                                'outcome2':'outcome4'})
            smach.StateMachine.add('BAR', Bar(),
                                   transitions={'outcome1':'FOO'})

        smach.StateMachine.add('SUB', sm_sub,
                               transitions={'outcome4':'outcome5'})

    # Execute SMACH plan
    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()
    sis2.stop()

if __name__ == '__main__':
    main()

#!/usr/bin/env python

import time

import rospy
import smach
import smach_ros


class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=('outcome1', 'outcome2'))

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        time.sleep(1)
        if COUNT < 3:
            global COUNT
            COUNT += 1
            return 'outcome1'
        else:
            return 'outcome2'


class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=('outcome2',))

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        time.sleep(1)
        return 'outcome2'


if __name__ == '__main__':

    rospy.init_node('smach_example_state_machine')

    COUNT = 0

    sm = smach.StateMachine(outcomes=('outcome4', 'outcome5'))
    with sm:
        smach.StateMachine.add(label='FOO', state=Foo(), transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
        smach.StateMachine.add(label='BAR', state=Bar(), transitions={'outcome2':'FOO'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()

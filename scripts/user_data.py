#!/usr/bin/env python
"""
Description:
    Create a two-state state machine where one state writes to userdata and
    the other state reads from userdata, and spews a message to rosout.

Usage:
    $> ./user_data.py

Output:
    [INFO] : State machine starting in initial state 'SET' with userdata:
        []
    [INFO] : State machine transitioning 'SET':'set_it'-->'GET'
    [INFO] : >>> GOT DATA! x = True
    [INFO] : State machine terminating 'GET':'got_it':'succeeded'
"""

import time

import rospy
import smach
import smach_ros


class Setter(smach.State):

    def __init__(self):
        super(Setter, self).__init__(outcomes=['set_it', "done"], input_keys=['count'], output_keys=['x', "count"])

    def execute(self, userdata):
        rospy.loginfo("Setter count = " + str(userdata.count))
        if userdata.count < 3:
            userdata.count += 1
            time.sleep(2)
            userdata.x = True
            return 'set_it'
        else:
            return "done"


class Getter(smach.State):

    def __init__(self):
        super(Getter, self).__init__(outcomes=['got_it'], input_keys=['x'])

    def execute(self, userdata):
        rospy.loginfo('Getter x = ' + str(userdata.x))
        time.sleep(1)
        return 'got_it'


if __name__ == '__main__':

    rospy.init_node('smach_example_user_data')

    sm = smach.StateMachine(outcomes=['succeeded'])
    sm.userdata.count = 0

    with sm:
        smach.StateMachine.add(label='SET', state=Setter(), transitions={'set_it': 'GET', "done": "succeeded"})
        smach.StateMachine.add(label='GET', state=Getter(), transitions={'got_it': 'SET'})

    sis = smach_ros.IntrospectionServer('smach_server', sm, '/ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()
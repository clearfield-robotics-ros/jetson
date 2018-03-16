#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16, Int16MultiArray

class Probe_State_Monitor:
    def __init__(self):
        rospy.init_node('probe_state_monitor')
        sub = rospy.Subscriber("/probe_status_reply", Int16MultiArray, update_probe_state)
        self.probe_current_state = 0

        r = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            print "what's going on?"
            r.sleep()

    def update_probe_state(data):
        self.probe_current_state = data.data

    def get_state(self):
        return self.probe_current_state

#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
import tf
import math
import pdb

#############################
### monitor current state ###

current_state = 0 # if we don't get msgs
def update_state(data):
    global current_state
    current_state = data.data

jetson_current_state = rospy.Subscriber('current_state', Int16, update_state)

###############################
### begin teensy simulation ###

probe_state = 0; # initial
#define ZERO  0
#define IDLE  1
#define PROBE 2
#define CALIB 3

def probeCmdClbk(data):
    global probe_state
    probe_state = data.data

    if probe_state == 2: # ie. change to probing
        finished_probing = False

def probe_contact(data):
    global finished_probing
    finished_probing = True

def main():
    rospy.init_node('probe_teensy_SIM')

    global br
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    max_probe_distance = rospy.get_param('max_probe_distance')
    probe_speed = rospy.get_param('probe_speed')

    global probe_distance
    probe_distance = 0

    global finished_probing
    finished_probing = True
    probe_contact_sub = rospy.Subscriber("probe_contact", Point, probe_contact)

    pub = rospy.Publisher("/probe_status_reply", Int16MultiArray, queue_size=10)
    pub = rospy.Publisher("/probe_contact_reply", Int16MultiArray, queue_size=10)
    sub = rospy.Subscriber("/probe_cmd_send", Int16, probeCmdClbk)

    r = rospy.Rate(100)  # 100 Hz

    while not rospy.is_shutdown():

        if not finished_probing:
            if probe_distance > max_probe_distance:
                finished_probing = True
            else:
                probe_distance += probe_speed

        elif finished_probing:

            if probe_distance > 0: # retract
                probe_distance -= probe_speed

        br.sendTransform((probe_distance,
            0,
            0),
            tf.transformations.quaternion_from_euler(0,0,0),
            rospy.Time.now(),
            "probe_tip",
            "probe_base")

        r.sleep()  # indent less when going back to regular gantry_lib


if __name__ == "__main__":
    main()

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
    global finished_probing
    global retracted_probe
    global foundObject
    if probe_state == 2: # ie. change to probing
        finished_probing = False
        retracted_probe = False
        foundObject = False


def probe_contact(data):

    global foundObject
    global finished_probing
    global contact_pub

    if not finished_probing:
        probe_contact_reply_msg = Int16MultiArray()
        probe_contact_reply_msg.data = [
            probe_state,                     # Echo: probe mode
            1,                               # Flag: probe initialization status
            int(finished_probing == 'True'), # Flag: probe complete status
            probe_distance,                  # Value: probe linear positon (mm)
            int(foundObject == 'True')]       # Flag: contact type
        contact_pub.publish(probe_contact_reply_msg)

    foundObject = True
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
    global retracted_probe
    retracted_probe = True
    global foundObject
    foundObject = False

    status_pub = rospy.Publisher("/probe_status_reply", Int16MultiArray, queue_size=10)
    global contact_pub
    contact_pub = rospy.Publisher("/probe_contact_reply", Int16MultiArray, queue_size=10)
    probe_contact_sub = rospy.Subscriber("probe_contact", Point, probe_contact)
    cmd_sub = rospy.Subscriber("/probe_cmd_send", Int16, probeCmdClbk)

    r = rospy.Rate(100) # 100 Hz

    while not rospy.is_shutdown():

        if not finished_probing:
            if probe_distance > max_probe_distance:
                finished_probing = True
            else:
                probe_distance += probe_speed

        elif finished_probing:
            if probe_distance > 0: # retract
                probe_distance -= probe_speed
            else:
                retracted_probe = True

        br.sendTransform((probe_distance,
            0,
            0),
            tf.transformations.quaternion_from_euler(0,0,0),
            rospy.Time.now(),
            "probe_tip",
            "probe_base")

        # Send out update every loop
        probe_status_reply_msg = Int16MultiArray()
        probe_status_reply_msg.data = [
            probe_state,                     # Echo: probe mode
            1,                               # Flag: probe initialization status
            int(retracted_probe),            # Flag: probe complete status
            probe_distance,                  # Value: probe linear positon (mm)
            int(foundObject)]                # Flag: contact type
        status_pub.publish(probe_status_reply_msg)

        r.sleep()  # indent less when going back to regular gantry_lib


if __name__ == "__main__":
    main()

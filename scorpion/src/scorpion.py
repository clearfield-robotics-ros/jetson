#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Point
from std_msgs.msg import Int16, Int16MultiArray
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import tf
import math
from probe.msg import probe_data
from gantry.msg import gantry_status;
from gantry.msg import to_gantry_msg;

def updateLocation(loc, rot):

	# send transform
    q_rot = tf.transformations.quaternion_from_euler(rot[0],rot[1],rot[2])
    br.sendTransform(loc,
         q_rot,
         rospy.Time.now(),
         "scorpion",
         "world")

    # send model to rviz
    m = Marker()
    m.header.frame_id = "world"
    m.header.seq = 0
    m.header.stamp = rospy.Time.now()
    m.ns = "scorpion"
    m.id = 0
    m.type = 10  # mesh
    m.mesh_resource = "package://scorpion/mesh/scorpion_master.STL";
    m.action = 0  # add/modify

    scopion_rot = [rot[0] + model_scorpion_offset_rot[0],
    	rot[1] + model_scorpion_offset_rot[1],
    	rot[2] + model_scorpion_offset_rot[2]]

    scopion_q_rot = tf.transformations.quaternion_from_euler(scopion_rot[0],scopion_rot[1],scopion_rot[2])
    m.pose.position.x = loc[0] + model_scorpion_offset_loc[0]
    m.pose.position.y = loc[1] + model_scorpion_offset_loc[1]
    m.pose.position.z = loc[2] + model_scorpion_offset_loc[2]
    m.pose.orientation.x = scopion_q_rot[0]
    m.pose.orientation.y = scopion_q_rot[1]
    m.pose.orientation.z = scopion_q_rot[2]
    m.pose.orientation.w = scopion_q_rot[3]

    m.scale.x = 1
    m.scale.y = 1
    m.scale.z = 1
    m.color.a = 1.0
    m.color.r = 0.5
    m.color.g = 0.5
    m.color.b = 0.5
    pub.publish(m)

### States ###
# 0 - Idle
# 1 - Calibrating
# 2 - General Surveying
# 3 - MD Pinpointing
# 4 - Probing
# 5 - Marking

def update_state(data):
    global current_state
    desired_state = data.data
    # print "GUI Desired State:", desired_state

    if desired_state == 3: # safety against awry metal detector signals
        if current_state == 2:
            current_state = desired_state
    else:
        current_state = desired_state


def update_probe_state(data):
    global probe_current_state
    probe_current_state = data


def update_gantry_state(data):
    global gantry_current_state
    gantry_current_state = data

    # global current_state
    # if data.calibration_flag == False and not current_state == 0:
    #     print("gantry calibration dropped out, what's going on!")
    #     current_state = 0
    # else:
    #     pass


### pub / sub ###
pub = rospy.Publisher('scorpion', Marker, queue_size=10)
jetson_current_state = rospy.Publisher('current_state', Int16, queue_size=10)
jetson_desired_state = rospy.Subscriber('desired_state', Int16, update_state)
gui_jetson_desired_state = rospy.Subscriber('/minebot_gui/minebot_gui/desired_state', Int16, update_state)
braking_desired_state = rospy.Publisher('braking_desired_state', Int16, queue_size=10)


def main():
    rospy.init_node('scorpion')

    # Transforms
    global br
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    # Parameters
    global model_scorpion_offset_loc, model_scorpion_offset_rot
    model_scorpion_offset_loc = rospy.get_param('model_scorpion_offset_loc')
    model_scorpion_offset_rot = rospy.get_param('model_scorpion_offset_rot')

    # States
    global current_state
    current_state = 0 # initial state

    rospy.Subscriber("/probe_teensy/probe_status_reply", probe_data, update_probe_state)
    global probe_current_state
    probe_current_state = probe_data()
    probe_cmd_pub = rospy.Publisher("/probe_teensy/probe_cmd_send", Int16, queue_size=10)

    rospy.Subscriber("/gantry_current_state", gantry_status, update_gantry_state);
    global gantry_current_state
    gantry_send_msg = to_gantry_msg()
    gantry_cmd_pub = rospy.Publisher("gantry_cmd_send", to_gantry_msg, queue_size=10)


    r = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():

        '''
        Loop Updates
        '''
        jetson_current_state.publish(current_state) # broadcast our state

        try:
            (loc,rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            updateLocation(loc,rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        '''
        State Machine
        '''
        if current_state == 0:
            braking_desired_state.publish(0) # no brakes while we're chilling

        elif current_state == 1:
            braking_desired_state.publish(1) # brakes on while calibrating

            print "Calibrating Gantry..."
            gantry_send_msg.state_desired = current_state
            gantry_cmd_pub.publish(gantry_send_msg)

            while not gantry_current_state.calibration_flag: # while not finished
                pass
            print "...Gantry Calibrated"


            print "Calibrating Probes..."
            probe_cmd_pub.publish(3) # start calibration
            rospy.sleep(0.5) # give time for handshake
            while not probe_current_state.init: # while not finished
                pass
            print "...Probes Calibrated"

            current_state = 0 # return to idle state

        elif current_state == 2:
            braking_desired_state.publish(0) # brakes off while surveying

        elif current_state == 3:
            braking_desired_state.publish(1) # brakes on while pinpointing

        elif current_state == 4:
            braking_desired_state.publish(1) # brakes on while probing for sure

        elif current_state == 5:
            braking_desired_state.publish(1) # brakes on while marking for sure

        else:
            print ("jetson is in an invalid state: see scorpion.py")

        r.sleep()

if __name__ == "__main__":
    main()

#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import tf
import math

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

# 0 - Idle
# 1 - Calibrating
# 2 - General Surveying
# 3 - MD Pinpointing
# 4 - Probing
# 5 - Marking
current_state = 0 # initial state

def update_state(data):
    global current_state
    desired_state = data.data

    if current_state == 0:
        current_state = desired_state

    elif current_state == 1:
        current_state = desired_state

    elif current_state == 2:
        current_state = desired_state

    elif current_state == 3:
        current_state = desired_state
 
    elif current_state == 4:
        if not desired_state == 3: # ie ignore new md signals onece probing
            current_state = desired_state

    elif current_state == 5:
        current_state = desired_state

    else:
        print ("invalid state")

def state_machine():

    global loc, rot

    if current_state == 0:
        braking_desired_state.publish(0) # no brakes while we're chilling

    elif current_state == 1:
        print "we're calibrating"
        braking_desired_state.publish(1) # brakes on while calibrating

    elif current_state == 2:
        braking_desired_state.publish(0) # brakes off while surveying 
        # loc[0] += 5 # advance!
        # updateLocation(loc, rot)

    elif current_state == 3:
        braking_desired_state.publish(1) # brakes on while pinpointing 

    elif current_state == 4:
        braking_desired_state.publish(1) # brakes on while probing for sure

    elif current_state == 5:
        braking_desired_state.publish(1) # brakes on while marking for sure

    else:
        print ("invalid state")

### pub / sub ###
pub = rospy.Publisher('scorpion', Marker, queue_size=10)
jetson_current_state = rospy.Publisher('current_state', Int16, queue_size=10)
jetson_desired_state = rospy.Subscriber('desired_state', Int16, update_state)
gui_jetson_desired_state = rospy.Subscriber('/minebot_gui/minebot_gui/desired_state', Int16, update_state)
braking_desired_state = rospy.Publisher('braking_desired_state', Int16, queue_size=10)

def main():
    
    rospy.init_node('scorpion')

    global br
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    
    global model_scorpion_offset_loc
    global model_scorpion_offset_rot
    model_scorpion_offset_loc = rospy.get_param('model_scorpion_offset_loc')
    model_scorpion_offset_rot = rospy.get_param('model_scorpion_offset_rot')

    # generate manually, but eventually get from localization!
    # global loc, rot
    # loc = [0,0,0]
    # rot = [0,0,0]
    # updateLocation(loc, rot)

    r = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():

        try:
            updateLocation(listener.lookupTransform('/odom', '/base_link', rospy.Time(0)))     
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        state_machine() # perform functions in the state

        jetson_current_state.publish(current_state) # broadcast our state

        r.sleep()  # indent less when going back to regular gantry_lib

if __name__ == "__main__":
    main()
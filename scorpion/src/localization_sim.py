#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import tf
import math

### monitor current state ###
current_state = 0 # if we don't get msgs
def update_state(data):
    global current_state
    current_state = data.data

jetson_current_state = rospy.Subscriber('current_state', Int16, update_state)


def updateLocation(loc, rot):

    if current_state == 2:
        loc[0] += 3 # advance!
    
	# send transform
    q_rot = tf.transformations.quaternion_from_euler(rot[0],rot[1],rot[2])
    br.sendTransform(loc,
         q_rot,
         rospy.Time.now(),
         "base_link", #"scorpion",
         "odom") #"world")


def main():
    
    rospy.init_node('localization_sim')

    global br
    br = tf.TransformBroadcaster()
    
    global loc, rot
    loc = [0,0,0]
    rot = [0,0,0]

    r = rospy.Rate(30)  # 30
    while not rospy.is_shutdown():

        updateLocation(loc, rot)

        r.sleep()  # indent less when going back to regular gantry_lib

if __name__ == "__main__":
    main()
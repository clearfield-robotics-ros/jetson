#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import tf
import math

braking_current_state = 0 # if we don't get msgs
def update_state(data):
    global braking_current_state, braking_current_state_pub
    braking_current_state = data.data
    braking_current_state_pub.publish(braking_current_state)

def main():

    rospy.init_node('braking_sim')

    global braking_current_state_pub
    braking_desired_state_sub = rospy.Subscriber('braking_desired_state', Int16, update_state)
    braking_current_state_pub = rospy.Publisher('braking_current_state', Int16, queue_size=10)

    rospy.spin()

if __name__ == "__main__":
    main()

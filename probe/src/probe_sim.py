#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
import tf

# in: command of sweeping / position
# out: position of gantry (geometry_msgs/Point)

# todo: second thread for restart

def setTarget(data):
    global target
    target = data
    print "Target", target

def main():

    global beginProbing
    beginProbing = false

    rospy.init_node('probe_sim')

    sub = rospy.Subscriber("/MDToProbe", Point, setTarget)

    r = rospy.Rate(0.01)  # 100 Hz

    while not rospy.is_shutdown():

        r.sleep()  # indent less when going back to regular gantry_lib


if __name__ == "__main__":
    main()
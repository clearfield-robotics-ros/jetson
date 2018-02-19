#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from visualization_msgs.msg import Marker
import tf
import math

# in: command of sweeping / position
# out: position of gantry (geometry_msgs/Point)

# todo: second thread for restart

seq = 0


def updateLocation(data):

    global seq

    # # make point for other ROS nodes
    # msg = PointStamped()
    # msg.point = Point(a, b, float(data.data))
    # msg2.header.frame_id = "world"
    # msg2.header.seq = seq
    # msg2.header.stamp = rospy.Time.now()
    # pub.publish(msg)

    # make marker for viz
    msg2 = Marker()
    msg2.header.frame_id = "world"
    msg2.header.seq = seq
    msg2.header.stamp = rospy.Time.now()
    msg2.ns = "scorpion"
    msg2.id = seq
    msg2.type = 10  # mesh
    msg2.mesh_resource = "package://scorpion/src/scorpion_master.STL";
    msg2.action = 0  # add/modify

    msg2.pose.position = data
    msg2.pose.orientation.x = 1.0
    msg2.pose.orientation.y = 0.0
    msg2.pose.orientation.z = 0.0
    msg2.pose.orientation.w = 1.0

    msg2.scale.x = 0.01
    msg2.scale.y = 0.01
    msg2.scale.z = 0.01
    msg2.color.a = 1.0
    msg2.color.r = 0.5
    msg2.color.g = 0.5
    msg2.color.b = 0.5
    pub.publish(msg2)

    # seq += 1


def main():

    global pub

    rospy.init_node('scorpion')
    pub = rospy.Publisher('scorpion', Marker, queue_size=10)
    rospy.sleep(1)

    r = rospy.Rate(1)  # 10 Hz


    pose = Point()
    pose.x = 0
    pose.y = 0
    pose.z = 0

    while not rospy.is_shutdown():

    	pose.x += 0.1
    	updateLocation(pose)

        r.sleep()  # indent less when going back to regular gantry_lib


if __name__ == "__main__":
    main()
#!/usr/bin/env python

import rospy
import numpy
import math
import tf
from std_msgs.msg import Int16
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class CylinderMine:
    def __init__(self, x, y, diameter):
        self.x = x
        self.y = y
        self.radius = diameter / 2.0

    def query(self, x_q, y_q):
        dist = numpy.sqrt((x_q - self.x)*(x_q - self.x) + (y_q - self.y)*(y_q - self.y))
        if dist < self.radius:
            return 15
        else:
            return 0


class CylinderMineExp:

    def __init__(self, x, y, diameter, decay):
        self.x = x
        self.y = y
        self.radius = diameter / 2.0  # in meters
        self.decay = decay  # in meters

        marker_pub = rospy.Publisher('mine_marker', Marker, queue_size=10)
        rospy.sleep(1)

        m = Marker()
        m.header.frame_id = "world"
        m.header.seq = 0
        m.header.stamp = rospy.Time.now()
        m.ns = "mine"
        m.id = 0
        m.type = 3  # cylinder
        m.action = 0  # add/modify
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = -150
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = diameter
        m.scale.y = diameter
        m.scale.z = 50
        m.color.a = 0.5
        m.color.r = 1.0
        m.color.g = 0.25
        m.color.b = 0.25
        marker_pub.publish(m)


    def query(self, x_q, y_q):
        dist = numpy.sqrt((x_q - self.x)*(x_q - self.x) + (y_q - self.y)*(y_q - self.y))
        if dist < self.radius:
            return 15
        ret = int(round(15*numpy.exp(-(dist - self.radius) / self.decay), 0))
        # print 15*numpy.exp(-(dist - self.radius) / self.decay)
        return ret


def query_mine(x,y):
    ret = mine.query(x,y)
    pub.publish(Int16(ret))


def main():
    rospy.init_node('mine_sim')
    global mine
    global pub

    landmine_pos = rospy.get_param('landmine_pos')
    landmine_diameter = rospy.get_param('landmine_diameter')
    mine = CylinderMineExp(landmine_pos[0], landmine_pos[1], landmine_diameter, 30) # 0.1

    listener = tf.TransformListener()
    pub = rospy.Publisher("md_signal", Int16, queue_size=10)

    r = rospy.Rate(100) # 100 Hz
    while not rospy.is_shutdown():
        
        try:
            (trans,rot) = listener.lookupTransform('/world', '/sensor_head', rospy.Time(0))
            query_mine(trans[0],trans[1])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # do things
        r.sleep()  # indent less when going back to regular gantry_lib


if __name__ == "__main__":
    main()
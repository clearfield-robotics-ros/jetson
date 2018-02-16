#!/usr/bin/env python

import rospy
import numpy
from std_msgs.msg import Int16
from geometry_msgs.msg import Point


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

    def query(self, x_q, y_q):
        dist = numpy.sqrt((x_q - self.x)*(x_q - self.x) + (y_q - self.y)*(y_q - self.y))
        if dist < self.radius:
            return 15
        ret = int(round(15*numpy.exp(-(dist - self.radius) / self.decay), 0))
        print 15*numpy.exp(-(dist - self.radius) / self.decay)
        return ret


def query_mine(data):
    ret = mine.query(data.x, data.y)
    pub.publish(Int16(ret))


def main():
    rospy.init_node('mine_sim')
    global mine
    global pub

    mine = CylinderMineExp(0.5, 0.25, 0.02, 0.1)

    sub = rospy.Subscriber("gantry_pos", Point, query_mine)
    pub = rospy.Publisher("md_signal", Int16, queue_size=10)

    r = rospy.Rate(100) # 100 Hz
    while not rospy.is_shutdown():
        # do things
        r.sleep()  # indent less when going back to regular gantry_lib


if __name__ == "__main__":
    main()
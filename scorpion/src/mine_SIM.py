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

    def __init__(self, idx, x, y, z, diameter, height, decay):
        self.x = x
        self.y = y
        self.z = z
        self.radius = diameter / 2.0  # in meters
        self.decay = decay  # in meters

        marker_pub = rospy.Publisher('mine_marker', Marker, queue_size=10)
        rospy.sleep(1)

        m = Marker()
        m.header.frame_id = "world"
        m.header.seq = idx
        m.header.stamp = rospy.Time.now()
        m.ns = "mine"
        m.id = idx
        m.type = 3  # cylinder
        m.action = 0  # add/modify
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z+height/2 # depth
        m.pose.orientation.x = 0.0
        m.pose.orientation.y = 0.0
        m.pose.orientation.z = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x = diameter
        m.scale.y = diameter
        m.scale.z = height
        m.color.a = 0.5
        m.color.r = 1.0
        m.color.g = 0.25
        m.color.b = 0.25
        marker_pub.publish(m)


    def query(self, x_q, y_q):
        dist = numpy.sqrt((x_q - self.x)*(x_q - self.x) + (y_q - self.y)*(y_q - self.y))
        if dist < self.radius:
            return 1500
        ret = int(round(1500*numpy.exp(-(dist - self.radius) / self.decay), 0))
        # print 15*numpy.exp(-(dist - self.radius) / self.decay)
        return ret

    def query_probe(self,x,y,z):

        z_bounds = False
        if z > self.z and z < (self.z + 50):
            z_bounds = True

        radius_bounds = False
        dist = numpy.sqrt((x - self.x)*(x - self.x) + (y - self.y)*(y - self.y))
        if dist < self.radius:
            radius_bounds = True

        return z_bounds and radius_bounds

def update_mine(data):
    global mine_index
    mine_index = data.data
    print "mine_index", mine_index

def main():
    rospy.init_node('mine_sim')

    listener = tf.TransformListener()
    pub = rospy.Publisher("md_signal", Int16, queue_size=10)
    pub2 = rospy.Publisher("probe_contact", Point, queue_size=10)

    landmine = rospy.get_param('landmine')
    landmine_burial_depth = rospy.get_param('landmine_burial_depth')
    landmine_diameter = rospy.get_param('landmine_diameter')
    landmine_height = rospy.get_param('landmine_height')
    metal_detector_decay = rospy.get_param('metal_detector_decay')

    # create mine objects
    mine_list = []
    for i in range(0,len(landmine)):
        mine = CylinderMineExp(i, landmine[i]['pos'][0], landmine[i]['pos'][1], landmine_burial_depth, landmine_diameter, landmine_height, metal_detector_decay)
        mine_list.append(mine)

    # init mine index
    jetson_desired_mine = rospy.Subscriber('/current_mine', Int16, update_mine)
    global mine_index
    mine_index = 0

    r = rospy.Rate(100)
    while not rospy.is_shutdown():

        ### Get MD Signal from Simulated Mine
        try:
            (t,R) = listener.lookupTransform('/world', '/md', rospy.Time(0))

            ret = mine_list[mine_index].query(t[0],t[1])
            pub.publish(Int16(ret))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        ### Get Probe Contact Points
        try:
            (t,R) = listener.lookupTransform('/world', '/probe_tip', rospy.Time(0))

            if mine_list[mine_index].query_probe(t[0],t[1],t[2]):
                p = Point(t[0],t[1],t[2])
                pub2.publish(p)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        r.sleep()

if __name__ == "__main__":
    main()

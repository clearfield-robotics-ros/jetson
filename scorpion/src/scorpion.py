#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from visualization_msgs.msg import Marker
import tf
import math

model_offset_rot = [math.pi/2, 0, math.pi]
model_offset_loc = [4.25,-4.25,0]

def updateLocation(loc, rot):

	# send transform
    q_rot = tf.transformations.quaternion_from_euler(rot[0],rot[1],rot[2])
    br.sendTransform(loc,
         q_rot,
         rospy.Time.now(),
         "scorpion_tf",
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

    scopion_rot = [rot[0] + model_offset_rot[0],
    	rot[1] + model_offset_rot[1],
    	rot[2] + model_offset_rot[2]]

    scopion_q_rot = tf.transformations.quaternion_from_euler(scopion_rot[0],scopion_rot[1],scopion_rot[2])
    m.pose.position.x = loc[0] + model_offset_loc[0]
    m.pose.position.y = loc[1] + model_offset_loc[1]
    m.pose.position.z = loc[2] + model_offset_loc[2]
    m.pose.orientation.x = scopion_q_rot[0]
    m.pose.orientation.y = scopion_q_rot[1]
    m.pose.orientation.z = scopion_q_rot[2]
    m.pose.orientation.w = scopion_q_rot[3]

    m.scale.x = 0.01
    m.scale.y = 0.01
    m.scale.z = 0.01
    m.color.a = 1.0
    m.color.r = 0.5
    m.color.g = 0.5
    m.color.b = 0.5
    pub.publish(m)


def main():

    global pub
    global br
    br = tf.TransformBroadcaster()

    rospy.init_node('scorpion')
    pub = rospy.Publisher('scorpion', Marker, queue_size=10)
    rospy.sleep(1)

    r = rospy.Rate(10)  # 10 Hz

    # generate manually, but eventually get from localization!

    loc = [0,0,0]
    rot = [0,0,0]


    while not rospy.is_shutdown():

    	loc[1] += 0.05 # advance!
    	updateLocation(loc, rot)

        r.sleep()  # indent less when going back to regular gantry_lib


if __name__ == "__main__":
    main()
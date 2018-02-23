#!/usr/bin/env python

# this takes input from the gantry and MD
# then it outputs position, detection pairs for strong detections

import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Point, PointStamped, Pose
from visualization_msgs.msg import Marker
import tf


prev_pos = (0, 0)
prev_forward = True
pos_history = []
detection_history = []
sweep_num = 1
seq = 0


def update_detection(data):
    global detection_history
    global pos_history
    global seq
    pos_history.append(prev_pos)
    detection_history.append(data.data)
    if data.data > 1:
        a = float(prev_pos[0])
        b = float(prev_pos[1])

        # make point for other ROS nodes
        msg = PointStamped()
        msg.point = Point(a, b, float(data.data))
        msg.header.frame_id = "world"
        msg.header.seq = seq
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)

        # make marker for viz
        msg2 = Marker()
        msg2.header = msg.header
        msg2.ns = "md_viz"
        msg2.id = seq
        msg2.type = 1  # cube
        msg2.action = 0  # add
        msg2.pose.position = Point(prev_world_pos[0], prev_world_pos[1], float(data.data)*10)
        msg2.pose.orientation.w = 1
        msg2.scale.x = 10
        msg2.scale.y = 10
        msg2.scale.z = 10
        msg2.color.a = 1.0
        msg2.color.r = 0.42
        msg2.color.g = 0.35
        msg2.color.b = 0.80
        pub2.publish(msg2)

        seq += 1
        print a, b, float(data.data)


def update_local_pos(x,y):
    global prev_pos
    prev_pos = (x,y)

def update_world_pos(x,y):
    global prev_world_pos
    prev_world_pos = (x,y)


def main():
    global pub
    global pub2
    rospy.init_node('md_analysis')

    sub = rospy.Subscriber('md_signal', Int16, update_detection)
    pub = rospy.Publisher('md_strong_signal', PointStamped, queue_size=10)
    pub2 = rospy.Publisher('md_viz', Marker, queue_size=10)

    listener = tf.TransformListener()

    r = rospy.Rate(100) # 100 Hz
    while not rospy.is_shutdown():
        
        try:
            (trans,rot) = listener.lookupTransform('/scorpion', '/sensor_head', rospy.Time(0))
            update_local_pos(trans[0],trans[1])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        try:
            (trans,rot) = listener.lookupTransform('/world', '/sensor_head', rospy.Time(0))
            update_world_pos(trans[0],trans[1])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # do things
        r.sleep()  # indent less when going back to regular gantry_lib

if __name__ == "__main__":
    main()
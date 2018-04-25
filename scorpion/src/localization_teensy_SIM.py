#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
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
        loc[0] += sim_walking_speed # advance!
        pass

	# send transform
    q_rot = tf.transformations.quaternion_from_euler(rot[0],rot[1],rot[2])
    # br.sendTransform(loc,
    #      q_rot,
    #      rospy.Time.now(),
    #      'base_link', #"scorpion",
    #      'world') #"world")

    #publish odometry/filtered
    sim_odom                            = Odometry()
    sim_odom.header.frame_id            = 'odom'
    sim_odom.header.stamp               = rospy.Time.now()
    sim_odom.child_frame_id             = 'base_link'
    sim_odom.pose.pose.position.x       = loc[0]
    sim_odom.pose.pose.position.y       = loc[1]
    sim_odom.pose.pose.position.z       = loc[2]
    sim_odom.pose.pose.orientation.x    = q_rot[0]    
    sim_odom.pose.pose.orientation.y    = q_rot[1]    
    sim_odom.pose.pose.orientation.z    = q_rot[2]
    sim_odom.pose.pose.orientation.w    = q_rot[3]

    global pub
    pub.publish(sim_odom)


def main():

    rospy.init_node('localization_sim')
    global pub
    pub = rospy.Publisher('odometry_filtered', Odometry, queue_size=10)

    global sim_walking_speed
    sim_walking_speed = rospy.get_param('sim_walking_speed')

    # global br
    # br = tf.TransformBroadcaster()

    global loc, rot
    loc = [0,0,0]
    rot = [0,0,0]

    r = rospy.Rate(30)  # 30
    while not rospy.is_shutdown():

        updateLocation(loc, rot)

        r.sleep()  # indent less when going back to regular gantry_lib

if __name__ == "__main__":
    main()

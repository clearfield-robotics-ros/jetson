#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
import tf
import math
import pdb

### monitor current state ###
current_state = 0 # if we don't get msgs
def update_state(data):
    global current_state
    current_state = data.data

jetson_current_state = rospy.Subscriber('current_state', Int16, update_state)


def setTarget(data):
    global target
    target = data
    print "target", target

def probe_to_gantry_transform(loc,rot):

    Hprobe = np.array([[1,0,0,loc.x],
                       [0,1,0,loc.y],
                       [0,0,1,loc.z],
                       [0,0,0,1]])

    Hyaw = np.array([[math.cos(rot.z),-math.sin(rot.z),0,0],
                     [math.sin(rot.z),math.cos(rot.z),0,0],
                     [0,0,1,0],
                     [0,0,0,1]])

    Hyrot = np.array([[math.cos(-probe_angle),0,math.cos(-probe_angle),0],
                      [0,1,0,0],
                      [math.sin(-probe_angle),0,math.cos(-probe_angle),0],
                      [0,0,0,1]])

    Hd = np.array([[1,0,0,-probe_length],
                   [0,1,0,0],
                   [0,0,1,0],
                   [0,0,0,1]])

    H = np.matmul(np.matmul(np.matmul(Hprobe,Hyaw),Hyrot),Hd)
    test = np.matmul(H,np.array([[0],[0],[0],[1]]))

    # br.sendTransform((test[0],test[1],test[2]),
    #     tf.transformations.quaternion_from_euler(0,0,0),
    #     rospy.Time.now(),
    #     "desired_gantry_pose",
    #     "gantry")

    (trans,rot) = listener.lookupTransform('/probe_base', '/sensor_head', rospy.Time(0))

    Hoffset = np.array([[1,0,0,-probe_base_offset_loc[0]],
                       [0,1,0,-probe_base_offset_loc[1]],
                       [0,0,1,-probe_base_offset_loc[2]],
                       [0,0,0,1]])

    H = np.matmul(np.matmul(np.matmul(np.matmul(Hoffset,Hprobe),Hyaw),Hyrot),Hd)
    trans = np.matmul(H,np.array([[0],[0],[0],[1]]))

    # br.sendTransform((trans[0],trans[1],trans[2]),
    #     tf.transformations.quaternion_from_euler(0,0,0),
    #     rospy.Time.now(),
    #     "desired_gantry_pose_offset",
    #     "gantry")

    return trans



### States ###
# 0 - Initial Search
# 1 - 45deg Search
# 2 - Max Info Search
probe_state = 0 # initial state

def main():
    rospy.init_node('probe_sim')

    global br
    br = tf.TransformBroadcaster()
    global listener
    listener = tf.TransformListener()

    landmine_pos = rospy.get_param('landmine_pos')
    landmine_diameter = rospy.get_param('landmine_diameter')
    landmine_height = rospy.get_param('landmine_height')
    global probe_base_offset_loc
    probe_base_offset_loc = rospy.get_param('probe_base_offset_loc')
    probe_safety_factor = rospy.get_param('probe_safety_factor')
    global probe_angle
    probe_angle = rospy.get_param('probe_base_offset_rot')[1]
    scorpion_gantry_offset_loc = rospy.get_param('scorpion_gantry_offset_loc')
    global probe_length
    probe_length = (scorpion_gantry_offset_loc[2] + probe_base_offset_loc[2] + abs(landmine_pos[2])) / math.sin(probe_angle)

    maxForwardSearch = math.cos(probe_angle)*landmine_height*(1/probe_safety_factor);


    null_target = Point()
    global target
    target = null_target
    set_desired_gantry_pose = False


    pub = rospy.Publisher("/gantry_probe_cmd", Twist, queue_size=10)
    sub = rospy.Subscriber("/set_probe_target", Point, setTarget)

    r = rospy.Rate(100)  # 100 Hz

    while not rospy.is_shutdown():

        # check we're in the right state and have a Target
        if current_state == 4 and not target == null_target:

            # get first probe Point - find x/y position for desired probe
            if probe_state == 0 and not set_desired_gantry_pose:

                # define desired probe tip position
                desired_probe_tip = Point()
                desired_probe_tip.x = target.x - landmine_diameter/2*probe_safety_factor
                desired_probe_tip.y = target.y
                desired_probe_tip.z = -300 + landmine_pos[2] # set to depth

                # br.sendTransform((desired_probe_tip.x,desired_probe_tip.y,desired_probe_tip.z),
                #     tf.transformations.quaternion_from_euler(0,0,0),
                #     rospy.Time.now(),
                #     "desired_probe_tip",
                #     "gantry")

                # we know our desired yaw angle
                desired_gantry_pose = Twist()
                desired_gantry_pose.angular.x = 0
                desired_gantry_pose.angular.y = 0
                desired_gantry_pose.angular.z = 0

                trans = probe_to_gantry_transform(desired_probe_tip, desired_gantry_pose.angular)

                desired_gantry_pose.linear.x = trans[0]
                desired_gantry_pose.linear.y = trans[1]
                desired_gantry_pose.linear.z = trans[2]

                print "desired_gantry_pose", desired_gantry_pose

                pub.publish(desired_gantry_pose)
                set_desired_gantry_pose = True


            elif probe_state == 1:
                print "get 2nd point"

            elif probe_state == 2:
                print "Fill in the gaps"

            # set state to that

            # wait for response - mine_sim function

            # determine intersection point




        r.sleep()  # indent less when going back to regular gantry_lib


if __name__ == "__main__":
    main()

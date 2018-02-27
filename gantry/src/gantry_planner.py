#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
import tf

# in: command of sweeping / position
# out: position of gantry (geometry_msgs/Point)

### monitor current state ###
current_state = 0 # if we don't get msgs
def update_state(data):
    global current_state
    current_state = data.data

jetson_current_state = rospy.Subscriber('current_state', Int16, update_state)

scorpion_gantry_offset_loc = rospy.get_param('scorpion_gantry_offset_loc')
scorpion_gantry_offset_rot = rospy.get_param('scorpion_gantry_offset_rot')
md_gantry_offset_loc = rospy.get_param('md_gantry_offset_loc')
probe_base_offset_loc = rospy.get_param('probe_base_offset_loc')
probe_base_offset_rot = rospy.get_param('probe_base_offset_rot')

def publish_transforms():
    global br

    br.sendTransform((scorpion_gantry_offset_loc[0],
        scorpion_gantry_offset_loc[1],
        scorpion_gantry_offset_loc[2]),
        tf.transformations.quaternion_from_euler(scorpion_gantry_offset_rot[0],
            scorpion_gantry_offset_rot[1],
            scorpion_gantry_offset_rot[2]),
        rospy.Time.now(),
        "gantry",
        "scorpion")

    br.sendTransform((sensor_head[0],sensor_head[1],sensor_head[2]),
        tf.transformations.quaternion_from_euler(sensor_head[3],sensor_head[4],sensor_head[5]),
        rospy.Time.now(),
        "sensor_head",
        "gantry")

    br.sendTransform((md_gantry_offset_loc[0],
        md_gantry_offset_loc[1],
        md_gantry_offset_loc[2]),
        tf.transformations.quaternion_from_euler(0,0,0),
        rospy.Time.now(),
        "md",
        "sensor_head")

    br.sendTransform((probe_base_offset_loc[0],
        probe_base_offset_loc[1],
        probe_base_offset_loc[2]),
        tf.transformations.quaternion_from_euler(probe_base_offset_rot[0],
            probe_base_offset_rot[1],
            probe_yaw_angle),
        rospy.Time.now(),
        "probe_base",
        "sensor_head")

    br.sendTransform((probe_distance,
        0,
        0),
        tf.transformations.quaternion_from_euler(0,0,0),
        rospy.Time.now(),
        "probe_tip",
        "probe_base")


def update_cmd(data):
    global cmd
    cmd = [data.x, data.y, data.z]


def update_cmd_probe(data):
    global cmd_probe
    cmd_probe = data


def main():
    rospy.init_node('gantry_sim')

    global br
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    global sensor_head
    sensor_head = [0]*6

    global cmd
    cmd = [0, 0, 0]
    sub = rospy.Subscriber("gantry_cmd_send", Point, update_cmd)
    global cmd_probe
    cmd_probe = Twist()
    sub2 = rospy.Subscriber("gantry_probe_cmd", Twist, update_cmd_probe)


    global probe_yaw_angle
    probe_yaw_angle = probe_base_offset_rot[2]
    global probe_distance
    probe_distance = 600

    trans = [0, 0, 0]
    rot = [0, 0, 0]
    vel_dir = 1

    gantry_width = rospy.get_param('gantry_width')
    gantry_sweep_speed = rospy.get_param('gantry_sweep_speed')
    low_lim = 0
    high_lim = gantry_width
    lat_vel = gantry_sweep_speed
    tolerance = 0.005

    rate = 100
    r = rospy.Rate(rate)  # 100 Hz

    while not rospy.is_shutdown():

        publish_transforms()

        ### idle ###
        if current_state == 0:
            pass

        ### sweeping ###
        elif current_state == 2:

            try:
                (trans,rot) = listener.lookupTransform('/gantry', '/sensor_head', rospy.Time(0))

                trans[1] += vel_dir * lat_vel / rate
                if trans[1] < low_lim + tolerance or trans[1] > high_lim - tolerance:
                    vel_dir *= -1

                sensor_head[0] = trans[0]
                sensor_head[1] = trans[1]
                sensor_head[2] = trans[2]
                sensor_head[3] = rot[0]
                sensor_head[4] = rot[1]
                sensor_head[5] = rot[2]

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        ### pinpointing ###
        elif current_state == 3:

            try:
                (trans,rot) = listener.lookupTransform('/gantry', '/sensor_head', rospy.Time(0))

                # convert desired md position to desired sensor_head position
                cmd[0] -= md_gantry_offset_loc[0]
                cmd[1] -= md_gantry_offset_loc[1]


                diff = [cmd[i] - trans[i] for i in range(3)]
                for i in range(3):
                    if abs(diff[i]) < (lat_vel / rate):
                        trans[i] = cmd[i]
                    else:
                        if diff[i] > 0:
                            trans[i] += (lat_vel / rate)
                        else:
                            trans[i] -= (lat_vel / rate)

                sensor_head[0] = trans[0]
                sensor_head[1] = trans[1]
                sensor_head[2] = trans[2]
                sensor_head[3] = rot[0]
                sensor_head[4] = rot[1]
                sensor_head[5] = rot[2]

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

        ### probe ###
        elif current_state == 4:
            print "probing"

            # probe_yaw_angle = 0
            # # probe_distance += 0.5
            #
            # sensor_head[0] = cmd_probe.linear.x
            # sensor_head[1] = cmd_probe.linear.y
            # sensor_head[2] = cmd_probe.linear.z
            # sensor_head[3] = cmd_probe.angular.x
            # sensor_head[4] = cmd_probe.angular.y
            # sensor_head[5] = cmd_probe.angular.z


        print "Cur: ", [round(val, 2) for val in trans]
        print "Cmd: ", cmd
        print "current_state: ", current_state
        print "-------------------"
        r.sleep()  # indent less when going back to regular gantry_lib


if __name__ == "__main__":
    main()

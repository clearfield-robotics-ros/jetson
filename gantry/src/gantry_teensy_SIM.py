#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
import tf
import math

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


def update_cmd(data):
    global cmd
    cmd = [data.x, data.y, data.z]

def update_gantry_desired_state(data):
    global gantry_desired_state
    gantry_desired_state = data.data


def main():
    rospy.init_node('gantry_sim')

    global br
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    # sensor head state
    global sensor_head
    sensor_head = [0]*6

    # TODO gotta fix these
    global cmd
    cmd = [0, 0, 0]
    sub = rospy.Subscriber("gantry_cmd_send", Point, update_cmd)

    # new MSG type
    # global gantry_desired_state
    # gantry_desired_state = Int16MultiArray()
    gantry_desired_state_sub = rospy.Subscriber("/gantry_desired_state", Int16MultiArray, update_gantry_desired_state)
    gantry_current_state_pub = rospy.Publisher("/gantry_current_state", Int16MultiArray, queue_size=10)
    desired_state_reached = True
    global probe_yaw_angle
    probe_yaw_angle = probe_base_offset_rot[2]

    trans = [0, 0, 0]
    rot = [0, 0, 0]
    vel_dir = 1
    gantry_width = rospy.get_param('gantry_width')
    gantry_sweep_speed = rospy.get_param('gantry_sweep_speed')
    low_lim = 0
    high_lim = gantry_width
    lat_vel = gantry_sweep_speed
    tolerance = 0.005

    rate = 50
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

            rate = 10

            if 'gantry_desired_state' in globals():

                diff = np.zeros(4)

                # X Position
                diff[0] = (gantry_desired_state[2] - sensor_head[0])/rate
                sensor_head[0] += diff[0]

                # Y Position
                diff[1] = (gantry_desired_state[3] - sensor_head[1])/rate
                sensor_head[1] += diff[1]

                # Gantry Yaw
                diff[2] = (float(gantry_desired_state[4])/180*math.pi - sensor_head[5])/rate
                sensor_head[5] += diff[2]

                # Probe Yaw
                diff[3] = (float(gantry_desired_state[5])/180*math.pi - probe_yaw_angle)/rate
                probe_yaw_angle += diff[3]

                if abs(np.sum(diff)) < 0.1:
                    desired_state_reached = True
                else:
                    desired_state_reached = False

        # Send out update every loop
        gantry_current_state_msg = Int16MultiArray()
        gantry_current_state_msg.data = [
            current_state,
            0, #current_sweep_velocity
            sensor_head[0], #current_x_position
            sensor_head[1], # current_y_position
            sensor_head[5]*180/math.pi, # current_yaw_angle
            probe_yaw_angle*180/math.pi, #current_probe_yaw_angle
            int(desired_state_reached)]
        gantry_current_state_pub.publish(gantry_current_state_msg)

        r.sleep()  # indent less when going back to regular gantry_lib


if __name__ == "__main__":
    main()

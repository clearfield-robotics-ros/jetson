#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
import tf

# in: command of sweeping / position
# out: position of gantry (geometry_msgs/Point)

def update_cmd(data):
    global cmd
    cmd = [data.x, data.y, data.z]


def update_mode(data):
    global mode
    mode = data.data


# def update_sensor_head_pos(trans,rot):
#     br.sendTransform((trans[0], trans[1], trans[2]),
#             tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2]),
#             rospy.Time.now(),
#             "sensor_head",
#             "gantry")   

def main():
    global mode
    global cmd
    sub = rospy.Subscriber("gantry_cmd_send", Point, update_cmd)
    sub2 = rospy.Subscriber("gantry_cmd_hack_send", Int16, update_mode)

    scorpion_gantry_offset_loc = rospy.get_param('scorpion_gantry_offset_loc')
    scorpion_gantry_offset_rot = rospy.get_param('scorpion_gantry_offset_rot')
    gantry_width = rospy.get_param('gantry_width')
    gantry_sweep_speed = rospy.get_param('gantry_sweep_speed')

    rospy.init_node('gantry_sim')

    global br
    br = tf.TransformBroadcaster()    
    listener = tf.TransformListener()

    mode = 0
    cmd = [0, 0, 0]

    trans = [0, 0, 0]
    rot = [0, 0, 0]
    vel_dir = 1
    
    low_lim = -gantry_width/2
    high_lim = gantry_width/2
    lat_vel = gantry_sweep_speed
    tolerance = 0.005

    rate = 100
    r = rospy.Rate(rate)  # 100 Hz

    while not rospy.is_shutdown():

        br.sendTransform((scorpion_gantry_offset_loc[0], 
                    scorpion_gantry_offset_loc[1], 
                    scorpion_gantry_offset_loc[2]),
                    tf.transformations.quaternion_from_euler(scorpion_gantry_offset_rot[0], 
                        scorpion_gantry_offset_rot[1], 
                        scorpion_gantry_offset_rot[2]),
                    rospy.Time.now(),
                    "gantry",
                    "scorpion")

        ### idle ###
        if mode == 0:
            pass

        ### sweeping ###
        elif mode == 2:

            print "sweeping"

            try:
                (trans,rot) = listener.lookupTransform('/gantry', '/sensor_head', rospy.Time(0))

                trans[1] += vel_dir * lat_vel / rate
                if trans[1] < low_lim + tolerance or trans[1] > high_lim - tolerance:
                    vel_dir *= -1

                br.sendTransform((trans[0], trans[1], trans[2]),
                    tf.transformations.quaternion_from_euler(rot[0], rot[1], rot[2]),
                    rospy.Time.now(),
                    "sensor_head",
                    "gantry")   

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):

                br.sendTransform((0,0,0),
                    tf.transformations.quaternion_from_euler(0,0,0),
                    rospy.Time.now(),
                    "sensor_head",
                    "gantry")  

                continue

        ### pinpointing ###
        elif mode == 3:

            print "pinpointing"

            try:
                (trans,rot) = listener.lookupTransform('/scorpion', '/sensor_head', rospy.Time(0))

                diff = [cmd[i] - trans[i] for i in range(3)]
                for i in range(3):
                    if abs(diff[i]) < (lat_vel / rate):
                        trans[i] = cmd[i]
                    else:
                        if diff[i] > 0:
                            trans[i] += (lat_vel / rate)
                        else:
                            trans[i] -= (lat_vel / rate)

                br.sendTransform((trans[0], 
                            trans[1], 
                            trans[2]),
                            tf.transformations.quaternion_from_euler(rot[0], 
                                rot[1], 
                                rot[2]),
                            rospy.Time.now(),
                            "sensor_head",
                            "scorpion")   

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


        print "Cur: ", [round(val, 2) for val in trans]
        print "Cmd: ", cmd
        print "Mode: ", mode
        print "-------------------"
        r.sleep()  # indent less when going back to regular gantry_lib


if __name__ == "__main__":
    main()
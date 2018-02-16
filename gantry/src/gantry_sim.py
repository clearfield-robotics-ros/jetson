#!/usr/bin/env python

import rospy
import numpy
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
import tf

# in: command of sweeping / position
# out: position of gantry (geometry_msgs/Point)

# todo: second thread for restart

def update_cmd(data):
    global cmd
    cmd = [data.x, data.y, data.z]


def update_mode(data):
    global mode
    mode = data.data


def main():
    global mode
    global cmd
    br = tf.TransformBroadcaster()
    mode = 0
    cmd = [0, 0, 0]

    cur_pos = [0, 0, 0]
    vel_dir = 1

    rospy.init_node('gantry_sim')

    sub = rospy.Subscriber("gantry_cmd_send", Point, update_cmd)
    sub2 = rospy.Subscriber("gantry_cmd_hack_send", Int16, update_mode)
    pub = rospy.Publisher("gantry_pos", Point, queue_size=10)
    pos_msg = Point()

    low_lim = 0.0
    high_lim = 1.0
    tolerance = 0.005
    long_vel = 0.3
    lat_vel = 2.0
    rate = 100
    r = rospy.Rate(rate/10)  # 100 Hz
    while not rospy.is_shutdown():
        if mode == 0:  # idle
            pass
        elif mode == 2:  # sweeping
            print "I know I'm sweeping"
            cur_pos[0] += vel_dir * lat_vel / rate
            cur_pos[1] += long_vel / rate
            if cur_pos[0] < low_lim + tolerance or cur_pos[0] > high_lim - tolerance:
                vel_dir *= -1
        elif mode == 3:  # positioning
            diff = [cmd[i] - cur_pos[i] for i in range(3)]
            for i in range(3):
                if abs(diff[i]) < (lat_vel / rate):
                    cur_pos[i] = cmd[i]
                else:
                    if diff[i] > 0:
                        cur_pos[i] += (lat_vel / rate)
                    else:
                        cur_pos[i] -= (lat_vel / rate)

        pos_msg.x = cur_pos[0]
        pos_msg.y = cur_pos[1]
        pos_msg.z = cur_pos[2]
        pub.publish(pos_msg)

        br.sendTransform((cur_pos[0], cur_pos[1], 0),
                         tf.transformations.quaternion_from_euler(0, 0, cur_pos[2]),
                         rospy.Time.now(),
                         "sensor_head",
                         "world")

        print "Cur: ", [round(val, 2) for val in cur_pos]
        print "Cmd: ", cmd
        print "Mode: ", mode
        print "-------------------"
        r.sleep()  # indent less when going back to regular gantry_lib


if __name__ == "__main__":
    main()
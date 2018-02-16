#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from gantry_lib_for_sim import Gantry

mode = 2
cmd = [0, 0, 0]


def update_cmd(data):
    print "got command"
    global mode
    global cmd
    if data.x < 0:
        # sweep!
        mode = 1
    else:
        # go to position
        mode = 2
        cmd = [data.x, data.y, data.z]


def main():
    rospy.init_node('gantry_driver')
    global mode
    global cmd

    sub = rospy.Subscriber("cmd_from_md", Point, update_cmd)

    g = Gantry()

    r = rospy.Rate(100) # 100 Hz
    while not rospy.is_shutdown():
        # do things
        if not g.initialized:
            print "Waiting for Gantry to Calibrate..."
        else:
            # normal operation
            if mode == 1:
                # sweep!
                g.send_state(2)
                print "Sweeping!"
            else:
                # go to position
                g.send_state(3)
                g.send_pos_cmd(cmd)
                print "positioning at " + str(cmd)
        r.sleep()  # indent less when going back to regular gantry_lib


if __name__ == "__main__":
    main()

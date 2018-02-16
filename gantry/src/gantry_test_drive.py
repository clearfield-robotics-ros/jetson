#!/usr/bin/env python

import rospy
from gantry_lib_for_sim import Gantry
from geometry_msgs.msg import Point


def main():
    rospy.init_node('gantry_driver')
    g = Gantry()

    # mode = 1  # sweep=1
    r = rospy.Rate(100)  # 100 Hz
    while not rospy.is_shutdown():
        # do things
        if not g.initialized:
            print "Waiting for Gantry to Calibrate..."
        else:
            # normal operation

            cmd_str = raw_input("floats for a new position, something else to sweep: ")
            try:
                cmd_str = cmd_str.split(" ")
                cmd = [0, 0,  0]
                for i in range(3):
                    cmd[i] = float(cmd_str[i])
                mode = 2
            except ValueError:
                mode = 1
            except IndexError:
                mode = 2

            if mode == 1:
                # sweep!
                g.send_state(2)
                print "send sweep"
            else:
                g.send_state(3)
                g.send_pos_cmd(cmd)
                print "send pos"
        r.sleep()


if __name__ == "__main__":
    main()

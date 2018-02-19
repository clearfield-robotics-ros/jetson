#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, Point
import numpy as np
import math
from copy import deepcopy


found_something = False
at_goal = False
cur_sig = np.array([0.0, 0.0, 0.0])
goal = np.array([0.0, 0.0])


def rotate(vec, angle):
    """Rotate a vector `v` by the given angle, relative to the anchor point."""
    x, y = vec

    cos_theta = math.cos(angle)
    sin_theta = math.sin(angle)

    nx = x*cos_theta - y*sin_theta
    ny = x*sin_theta + y*cos_theta

    return np.array([nx, ny])


def update_pos(data):
    global found_something
    global cur_sig
    global at_goal

    # print "updating pos"
    # print data.point

    if data.point.z > 6:
        # print "found something!"
        found_something = True

    cur_sig[0] = data.point.x
    cur_sig[1] = data.point.y
    cur_sig[2] = data.point.z

    if np.linalg.norm(cur_sig[:2] - goal) < 0.01:
        at_goal = True
    else:
        at_goal = False


rospy.init_node('md_planner')
pub = rospy.Publisher('/cmd_from_md', Point, queue_size=10)
sendToProbe = rospy.Publisher('/MDToProbe', Point, queue_size=10)
sub = rospy.Subscriber('md_strong_signal', PointStamped, update_pos)



def set_and_wait_for_goal(my_goal):
    global goal
    global at_goal
    global pub

    # print "setting!"

    at_goal = False
    goal = my_goal

    msg = Point()
    msg.x = my_goal[0]
    msg.y = my_goal[1]
    msg.z = 0

    r = rospy.Rate(3)
    while not at_goal:
        pub.publish(msg)
        # print "not as goal"
        r.sleep()
    else:
        return deepcopy(cur_sig)


def main():
    global pub

    sweep_msg = Point(-1.0, 0, 0)

    r = rospy.Rate(100)  # 100 Hz
    while not rospy.is_shutdown():
        if not found_something:
            # print "sweeping"
            pub.publish(sweep_msg)
        else:
            print "pinpointing!"
            #pub.publish(Point(cur_sig[0], cur_sig[1], 0))
            done = False
            step_size = 0.04
            shift = np.array([step_size, 0])
            shift_2 = rotate(shift, 2 * math.pi / 3)
            shift_3 = rotate(shift, 4 * math.pi / 3)

            while not done:
                print "--------------------------------------------------"
                # start pinpointing starting at the current location
                # try three points
                # check if done
                # if not done, move in a new direction

                cur_pos = deepcopy(cur_sig[:2])
                cur_sig_pow = deepcopy(cur_sig[2])
                point_one = cur_pos + shift
                point_two = cur_pos + shift_2
                point_three = cur_pos + shift_3

                print cur_pos, point_one, point_two, point_three

                p1 = set_and_wait_for_goal(point_one)
                print p1
                p2 = set_and_wait_for_goal(point_two)
                print p2
                p3 = set_and_wait_for_goal(point_three)
                print p3

                v1 = p3 - p1
                v2 = p2 - p1
                cp2 = np.cross(v1, v2)[:2]
                sigs = [p1[2], p2[2], p3[2]]
                grad_factor = (max(sigs) - min(sigs)) / 10.0
                print "grad_factor", grad_factor
                if grad_factor < 0.01:
                    done = True
                    print cur_pos
                else:
                    # print v1, v2, cp2
                    grad = step_size * grad_factor * cp2 / np.linalg.norm(cp2)
                    print "cp2 norm", np.linalg.norm(cp2)
                    print "grad", grad

                    new_point = cur_pos + grad
                    print "new_point", new_point
                    p_new = set_and_wait_for_goal(new_point)
                    if p_new[2] == cur_sig_pow:
                        done = True
            else:

                # pass the batton to the probe
                msg = Point(cur_sig[0], 
                            cur_sig[1], 
                            cur_sig[2])
                sendToProbe.publish(msg)

                print "cur_sig", cur_sig
                return
        r.sleep()

if __name__ == "__main__":
    main()

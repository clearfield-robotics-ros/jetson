#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Int16
from std_msgs.msg import String
from gantry.msg import gantry_status
import numpy as np
import math
from copy import deepcopy


marker_pub = rospy.Publisher('md_detection', Marker, queue_size=10)
def visualize_final_point(x,y,z,col):
    # Visualize probe point
    global marker_pub
    msg = Marker()
    msg.header.frame_id = "gantry"
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.ns = "md_detection"
    msg.id = 0
    msg.type = 0  # arrow
    msg.action = 0  # add
    msg.pose.position = Point(x,y,z+50)

    q_rot = tf.transformations.quaternion_from_euler(0,1.5708,0)
    msg.pose.orientation.x = q_rot[0]
    msg.pose.orientation.y = q_rot[1]
    msg.pose.orientation.z = q_rot[2]
    msg.pose.orientation.w = q_rot[3]

    msg.scale.x = 50
    msg.scale.y = 10
    msg.scale.z = 10
    msg.color.a = 1.0
    msg.color.r = col[0]
    msg.color.g = col[1]
    msg.color.b = col[2]
    marker_pub.publish(msg)

lims_set = False
found_something = False
at_goal = False
cur_sig = np.array([0.0, 0.0, 0.0])
goal = np.array([0.0, 0.0])
collect_data = False
data_collected = []
dist = 0
x_lims = [0, 1000]
y_lims = [0, 1000]
cur_state = 0

def rotate(vec, angle):
    """Rotate a vector `v` by the given angle, relative to the anchor point."""
    x, y = vec

    cos_theta = math.cos(angle)
    sin_theta = math.sin(angle)

    nx = x*cos_theta - y*sin_theta
    ny = x*sin_theta + y*cos_theta

    return np.array([nx, ny])


def incoming_signal(data):
    global found_something
    global cur_sig
    global at_goal
    global data_collected
    global dist

    # print "updating pos"
    # print data.point

    if data.point.z > 1200 and cur_state > 1:
        jetson_desired_state.publish(3)  # start pinpointing
        found_something = True

    cur_sig[0] = data.point.x
    cur_sig[1] = data.point.y
    cur_sig[2] = data.point.z

    if collect_data:
        data_collected.append(deepcopy(cur_sig))
    else:
        data_collected = []

    if np.linalg.norm(cur_sig[:2] - goal) < 1.0:
        at_goal = True
    else:
        at_goal = False
        dist = np.linalg.norm(cur_sig[:2] - goal)


def update_lims(data):
    global lims_set
    global x_lims
    global y_lims
    global cur_state
    x_lims = [data.x_min+1.0, data.x_max-1.0]
    y_lims = [data.y_min+1.0, data.y_max-1.0]
    cur_state = data.state
    lims_set = True


# pubs & subs
rospy.init_node('md_planner')
jetson_desired_state = rospy.Publisher('/desired_state', Int16, queue_size=10)
pub = rospy.Publisher('/cmd_from_md', Point, queue_size=10)
sendToProbe = rospy.Publisher('/set_probe_target', Point, queue_size=10)
sub = rospy.Subscriber('md_strong_signal', PointStamped, incoming_signal)

sub2 = rospy.Subscriber('gantry_current_status', gantry_status, update_lims)
listener = tf.TransformListener()


def set_and_wait_for_goal(my_goal, collect):
    global goal
    global at_goal
    global pub
    global collect_data
    global gantry_sweep_angle
    global sensorhead_md_offset_loc

    # print "setting!"

    at_goal = False
    goal = my_goal

    print "my_goal",my_goal

    msg = Point()
    msg.x = my_goal[0]
    msg.y = my_goal[1]
    msg.z = gantry_sweep_angle

    collect_data = collect

    r = rospy.Rate(3)
    while not at_goal:
        # print "distance away", dist
        pub.publish(msg)
        # print "not as goal"
        r.sleep()
    else:
        return deepcopy(data_collected)


def limit_val(val, lims):
    if val < lims[0]:
        return lims[0]
    elif val > lims[1]:
        return lims[1]
    else:
        return val


def main():
    global pub
    global found_something

    global gantry_sweep_angle
    gantry_sweep_angle = rospy.get_param('gantry_sweep_angle')
    global sensorhead_md_offset_loc
    sensorhead_md_offset_loc = rospy.get_param('sensorhead_md_offset_loc')
    scorpion_gantry_offset_loc = rospy.get_param('scorpion_gantry_offset_loc')

    sweep_msg = Point(-1e6, 0, 0)
    # x_lims = np.array([0, 1000])
    # y_lims = np.array([0, 1000])
    within = 0.01  # within 10% of max

    reached_sweeping_pos = False

    r = rospy.Rate(100)  # 100 Hz
    while not rospy.is_shutdown():

        if not found_something:
            # print "not found"
            if not lims_set:
                # print "no lims"
                pass
            elif not reached_sweeping_pos:
                # print "go to sweep pos"
                # TODO send to sweeping position, once
                sweep_pos = [100, cur_sig[1]]
                set_and_wait_for_goal(sweep_pos, collect=False)
                # print "got to sweep pos"
                reached_sweeping_pos = True
            else:
                # print "sweep cmd"
                pub.publish(sweep_msg)
        else:

            # do x check
            print "Start x check"
            cur_pos = deepcopy(cur_sig[:2])
            print cur_pos

            # create positive and negative position goals
            plus_val = limit_val(cur_pos[0] + 90.0, x_lims)
            minus_val = limit_val(cur_pos[0] - 90.0, x_lims)
            # print plus_val, minus_val
            plus_pos = deepcopy(cur_pos)
            plus_pos[0] = plus_val
            minus_pos = deepcopy(cur_pos)
            minus_pos[0] = minus_val
            print plus_pos, minus_pos

            # do movements, collect data, determine peak
            set_and_wait_for_goal(plus_pos, collect=False)
            collected = set_and_wait_for_goal(minus_pos, collect=True)
            max_sig = max(collected, key=lambda x: x[2])
            print max_sig
            filtered_collected = [pos[0] for pos in collected if pos[2] > (1 - within) * max_sig[2]]
            # print filtered_collected
            print np.mean(filtered_collected)
            start_from = [np.mean(filtered_collected), cur_pos[1]]

            # use new goal and move in y now
            set_and_wait_for_goal(start_from, collect=False)
            print "Start y check"
            cur_pos = deepcopy(cur_sig[:2])
            print cur_pos

            # create positive and negative position goals
            plus_val = limit_val(cur_pos[1] + 90.0, y_lims)
            minus_val = limit_val(cur_pos[1] - 90.0, y_lims)
            # print plus_val, minus_val
            plus_pos = deepcopy(cur_pos)
            plus_pos[1] = plus_val
            minus_pos = deepcopy(cur_pos)
            minus_pos[1] = minus_val
            print plus_pos, minus_pos

            # do movements, collect data, determine peak
            set_and_wait_for_goal(plus_pos, collect=False)
            collected = set_and_wait_for_goal(minus_pos, collect=True)
            max_sig = max(collected, key=lambda x: x[2])
            print max_sig
            filtered_collected = [pos[1] for pos in collected if pos[2] > (1 - within) * max_sig[2]]
            # print filtered_collected
            print np.mean(filtered_collected)
            start_from = [cur_pos[0], np.mean(filtered_collected)]

            # pass the batton to the probe
            x_offset = + math.sin(gantry_sweep_angle)*sensorhead_md_offset_loc[1] - math.cos(gantry_sweep_angle)*sensorhead_md_offset_loc[0]
            y_offset = - math.sin(gantry_sweep_angle)*sensorhead_md_offset_loc[0] - math.cos(gantry_sweep_angle)*sensorhead_md_offset_loc[1]

            msg = Point(max_sig[0] - x_offset,
                        max_sig[1] - y_offset,
                        max_sig[2])
            sendToProbe.publish(msg)

            visualize_final_point(max_sig[0] - x_offset,
                                  max_sig[1] - y_offset,
                                  -scorpion_gantry_offset_loc[2], [1,0,0])

            raw_input("\nPress Enter to continue...\n")

            jetson_desired_state.publish(4)

            # reset for next cycle
            found_something = False
            reached_sweeping_pos = False

        r.sleep()

if __name__ == "__main__":
    main()

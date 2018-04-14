#!/usr/bin/env python

import rospy
import tf
import math;
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Int16
from gantry.msg import gantry_status;
from gantry.msg import to_gantry_msg;

#JETSON
current_state               = 0;            # status of jetson

#GANTRY
md_cmd                      = to_gantry_msg();
probe_cmd                   = to_gantry_msg();

### ------------------ PARAMETERS ---------------------------- ###

sensorhead_md_offset_loc    = rospy.get_param('sensorhead_md_offset_loc');      #mm
gantry_sweep_speed          = rospy.get_param('gantry_sweep_speed');            #mm/s

### -------------------- monitor current state --------------- ###

def update_state(data):
    global current_state;
    current_state           = data.data;

### ---------------------------------------------------------- ###

def update_md_cmd(data):
    global md_cmd;
    global gantry_sweep_speed;
    global sensorhead_md_offset_loc;

    if (data.x <= -1e6):
        #sweep
        md_cmd.state_desired        = 2;
        md_cmd.sweep_speed_desired  = gantry_sweep_speed;
        md_cmd.x_desired            = 0;
        md_cmd.y_desired            = 0;
        md_cmd.yaw_desired          = 0;
    else:
        #pin pointing
        md_cmd.state_desired        = 3;
        md_cmd.sweep_speed_desired  = gantry_sweep_speed;
        md_cmd.x_desired            = data.x;
        md_cmd.y_desired            = data.y;
        md_cmd.yaw_desired          = data.z;

def update_probe_cmd(data):
    global probe_cmd;
    probe_cmd = data;

### ---------------------------------------------------------- ###

def draw_bounds(x_min,x_max,y_min,y_max):
    global gantry_bounds_viz_pub
    msg = Marker()
    msg.header.frame_id = "gantry"
    msg.id = 1
    msg.header.seq = 1
    msg.header.stamp = rospy.Time.now()
    msg.ns = "gantry_bounds_viz"
    msg.type = msg.LINE_STRIP  # line strip
    msg.action = msg.ADD  # add
    msg.pose.orientation.w = 1
    msg.scale.x = 2.
    msg.scale.y = 2.
    msg.scale.z = 2.
    msg.color.a = 1.
    msg.color.r = 1.
    msg.color.g = 0.
    msg.color.b = 0.
    msg.points = []
    msg.points.append(Point(x_min,y_min,0))
    msg.points.append(Point(x_max,y_min,0))
    msg.points.append(Point(x_max,y_max,0))
    msg.points.append(Point(x_min,y_max,0))
    msg.points.append(Point(x_min,y_min,0))
    gantry_bounds_viz_pub.publish(msg)

def update_gantry_state(data):
    draw_bounds(data.x_min,data.x_max,data.y_min,data.y_max)

### ---------------------------------------------------------- ###

def main():
    global current_state;
    global md_cmd;
    global probe_cmd;
    rospy.init_node('gantry_planner');

    listener = tf.TransformListener()

    # from jetson
    jetson_current_state    = rospy.Subscriber('current_state', Int16, update_state);
    # from metal detector
    md_subscriber           = rospy.Subscriber("/cmd_from_md", Point, update_md_cmd);
    # from probe
    gantry_desired_state    = rospy.Subscriber("/cmd_from_probe", to_gantry_msg, update_probe_cmd);

    gantry_send_msg = to_gantry_msg()
    gantry_cmd_pub = rospy.Publisher("gantry_cmd_send", to_gantry_msg, queue_size=10)

    global gantry_bounds_viz_pub
    gantry_bounds_viz_pub = rospy.Publisher('gantry_bounds_viz', Marker, queue_size=10)
    rospy.Subscriber("/gantry_current_status", gantry_status, update_gantry_state)

    r = rospy.Rate(50)
    while not rospy.is_shutdown():

        # idle
        if current_state == 0:
            pass;

        # calibration
        elif current_state == 1:
            pass
            # gantry_send_msg.state_desired = current_state
            # gantry_cmd_pub.publish(gantry_send_msg)
            # print ("Calibrating!");

        # sweeping
        elif current_state == 2:
            gantry_send_msg.state_desired = current_state
            gantry_send_msg.sweep_speed_desired = 0         # TODO
            gantry_cmd_pub.publish(gantry_send_msg)
            print ("Sweeping!");

        # pin pointing, listening to MD
        elif current_state == 3:
            gantry_cmd_pub.publish(md_cmd)
            print "positioning at " + str(md_cmd);

        # probing, listening to PROBE
        elif current_state == 4:
            gantry_cmd_pub.publish(probe_cmd)
            print "Probing at " + str(probe_cmd);

        r.sleep()  # indent less when going back to regular gantry_lib

if __name__ == "__main__":
    main()

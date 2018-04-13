#!/usr/bin/env python

import rospy
import tf
import math;
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from gantry.msg import gantry_status;
from gantry.msg import to_gantry_msg;

#JETSON
current_state               = 0;            # status of jetson
desired_state               = 0;            # what we want the jetson to be in

#GANTRY
md_cmd                      = to_gantry_msg();
probe_cmd                   = to_gantry_msg();
desired_state_reached       = False;

### ------------------ TRANSFORMS ---------------------------- ###

gantry_md_offset_loc        = rospy.get_param('gantry_md_offset_loc');      #mm
probe_base_offset_loc       = rospy.get_param('probe_base_offset_loc');     #mm
probe_base_offset_rot       = rospy.get_param('probe_base_offset_rot');     #rad
probe_yaw_angle             = probe_base_offset_rot[2];                     #rad
gantry_sweep_speed          = rospy.get_param('gantry_sweep_speed');        #mm/s

### -------------------- monitor current state --------------- ###

def update_state(data):
    global current_state;
    current_state           = data.data;

### ---------------------------------------------------------- ###

def update_md_cmd(data):
    global md_cmd;
    global gantry_sweep_speed;
    global gantry_md_offset_loc;
    global probe_yaw_angle;

    if (data.x < 0):
        #sweep
        md_cmd.state_desired        = 2;
        md_cmd.sweep_speed_desired  = gantry_sweep_speed;
        md_cmd.x_desired            = 0;
        md_cmd.y_desired            = 0;
        md_cmd.yaw_desired          = 0;
        md_cmd.probe_angle_desired  = 0;
    else:
        #pin pointing
        md_cmd.state_desired        = 3;
        md_cmd.sweep_speed_desired  = gantry_sweep_speed;
        md_cmd.x_desired            = data.x - gantry_md_offset_loc[0];
        md_cmd.y_desired            = data.y - gantry_md_offset_loc[1];
        md_cmd.yaw_desired          = -1.09956#data.z;
        md_cmd.probe_angle_desired  = probe_yaw_angle;

def update_probe_cmd(data):
    global probe_cmd
    probe_cmd = data

def main():
    global current_state;
    global md_cmd;
    global probe_cmd;
    global desired_state_reached;
    global probe_base_offset_rot;
    global probe_yaw_angle;
    probe_yaw_angle = probe_base_offset_rot[2];

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

    r = rospy.Rate(50)
    while not rospy.is_shutdown():

        # idle
        if current_state == 0:
            pass;

        # calibration
        elif current_state == 1:
            gantry_send_msg.state_desired = current_state
            gantry_cmd_pub.publish(gantry_send_msg)
            print ("Calibrating!");

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

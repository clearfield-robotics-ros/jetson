#!/usr/bin/env python

import rospy
import tf
import math;
import numpy as np
from gantry.msg import gantry_status;
from gantry.msg import to_gantry_msg;

### ---------------------------- Preset parameters --------------------------------- ###
rate                        = 50;
gantry_width                = rospy.get_param('gantry_width')
gantry_sweep_speed          = rospy.get_param('gantry_sweep_speed')
gantry_sweep_angle          = rospy.get_param('gantry_sweep_angle')
gantry_trans_speed          = gantry_sweep_speed;
gantry_rot_speed            = rospy.get_param('gantry_rot_speed');     #rad per second

### ---------------------------- Parameters that are updated ------------------------ ###

gantry_cmd                  = to_gantry_msg();
sensor_head                 = [0]*6;
vel_dir                     = 1;
desired_state_reached       = False;
gantry_calib_flag           = False;

### --------------------------- Updating the modes and commands -------------------- ###

def update_gantry_cmd(data):
    global gantry_cmd;
    gantry_cmd = data;

### --------------------------------- Publish Transforms --------------------------- ###

def publish_transforms():
    global br;
    global sensor_head;

    # sent by teensy/sim
    br.sendTransform((sensor_head[0],sensor_head[1],sensor_head[2]),
        tf.transformations.quaternion_from_euler(sensor_head[3],sensor_head[4],sensor_head[5]),
        rospy.Time.now(),
        "sensor_head",
        "gantry")

### ------------------------------ Actuating ------------------------------------- ###

def sweep():
    global sweep_speed;
    global sensor_head;
    global gantry_width;
    global gantry_sweep_speed;
    global vel_dir;

    tolerance           = 0.005;
    rate                = 100;

    low_lim             = 0;
    high_lim            = gantry_width;
    lat_vel             = gantry_sweep_speed * 10; #cm to mm

    #normally we will check if they are out of origin
    #move them slowly back, before we start sweeping
    #for simplicity we will just zero them instantly
    sensor_head[0] = 0;
    sensor_head[2] = 0;
    sensor_head[3] = 0;
    sensor_head[4] = 0;
    sensor_head[5] = gantry_sweep_angle; # we are no longer sweeping at zero

    sensor_head[1] += vel_dir * lat_vel / rate;
    if sensor_head[1] < low_lim + tolerance or sensor_head[1] > high_lim - tolerance:
        vel_dir *= -1;


def actuate_to_desired():
    global sensor_head;
    global rate;
    global gantry_cmd;
    global sensor_head;
    global gantry_trans_speed;
    global gantry_rot_speed;
    global desired_state_reached;

    lat_vel = gantry_sweep_speed * 10; #cm to mm
    diff    = np.zeros(4);

    trans_mm_per_loop   = gantry_trans_speed / float(rate);
    rot_rad_per_loop    = gantry_rot_speed / float(rate);

    # move each DoF closer to desired by 1 step
    desired_state_reached_flag  = 0;
    # x position
    x_diff          = gantry_cmd.x_desired - sensor_head[0];
    x_dir           = np.sign(x_diff);
    if (abs(x_diff) <= trans_mm_per_loop):
        sensor_head[0]  = gantry_cmd.x_desired;
        desired_state_reached_flag += 1;
    else:
        sensor_head[0] += x_dir * trans_mm_per_loop;

    # y position
    y_diff          = gantry_cmd.y_desired - sensor_head[1];
    y_dir           = np.sign(y_diff);
    if (abs(y_diff) <= trans_mm_per_loop):
        sensor_head[1]  = gantry_cmd.y_desired;
        desired_state_reached_flag += 1;
    else:
        sensor_head[1] += y_dir * trans_mm_per_loop;

    # yaw
    yaw_diff        = gantry_cmd.yaw_desired - sensor_head[5];
    yaw_dir         = np.sign(yaw_diff);
    if (abs(yaw_diff) <= rot_rad_per_loop):
        sensor_head[5]  = gantry_cmd.yaw_desired;
        desired_state_reached_flag += 1;
    else:
        sensor_head[5] += yaw_dir * rot_rad_per_loop;

    # check if 3 DOFs reached position
    if (desired_state_reached_flag == 3):
        desired_state_reached   = True;
    else:
        desired_state_reached   = False;

### ----------------------------------------------------------------------------- ###

def main():
    global gantry_cmd;
    global sensor_head;
    global desired_state_reached;
    global gantry_calib_flag;

    rospy.init_node('gantry_teensy_SIM')

    global br
    br = tf.TransformBroadcaster()

    # update mode
    gantry_cmd_sub           = rospy.Subscriber("gantry_cmd_send", to_gantry_msg, update_gantry_cmd);
    # from gantry
    gantry_current_status_pub = rospy.Publisher("/gantry_current_status", gantry_status, queue_size=1);

    gantry_x_max = rospy.get_param('gantry_x_max')
    gantry_y_max = rospy.get_param('gantry_y_max')

    r = rospy.Rate(50);

    while not rospy.is_shutdown():

        ### idle ###
        if gantry_cmd.state_desired == 0:
            pass

        ### calibrate ###
        elif gantry_cmd.state_desired == 1:
            rospy.sleep(1)
            gantry_calib_flag = True;

        ### sweeping ###
        elif gantry_cmd.state_desired == 2:
            sweep();

        ### moving to position ###
        elif gantry_cmd.state_desired == 3:
            actuate_to_desired();

        # gantry teensy only publishes transforms
        publish_transforms();

        ### ------------- PREPARE MESSAGES TO PUBLISH ---------------- ###

        #prepare the messages
        gantry_current_status_msg = gantry_status()
        gantry_current_status_msg.state = gantry_cmd.state_desired
        gantry_current_status_msg.sweep_speed = 0                 # TODO
        gantry_current_status_msg.x = sensor_head[0]
        gantry_current_status_msg.y = sensor_head[1]
        gantry_current_status_msg.yaw = sensor_head[5]            # rad
        gantry_current_status_msg.position_reached = desired_state_reached
        gantry_current_status_msg.calibration_flag = gantry_calib_flag;
        gantry_current_status_msg.x_min = 0
        gantry_current_status_msg.x_max = gantry_x_max
        gantry_current_status_msg.y_min = 0
        gantry_current_status_msg.y_max = gantry_y_max

        # publish the messages
        gantry_current_status_pub.publish(gantry_current_status_msg);

        r.sleep();

if __name__ == "__main__":
    main()

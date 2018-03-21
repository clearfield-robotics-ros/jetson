#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
import tf
from gantry.msg import gantry_status;
from gantry.msg import to_gantry_msg;
import math;

# scorpion_gantry_offset_loc = rospy.get_param('scorpion_gantry_offset_loc')
# scorpion_gantry_offset_rot = rospy.get_param('scorpion_gantry_offset_rot')
# md_gantry_offset_loc = rospy.get_param('md_gantry_offset_loc')
# probe_base_offset_loc = rospy.get_param('probe_base_offset_loc')
# probe_base_offset_rot = rospy.get_param('probe_base_offset_rot')

### monitor current state ###
current_state = 0 # if we don't get msgs
def update_state(data):
    global current_state
    current_state = data.data

jetson_current_state = rospy.Subscriber('current_state', Int16, update_state)

### ----------------------------- TRANSFORMS --------------------------------------- ###

#setup params
scorpion_gantry_offset_loc  = rospy.get_param('scorpion_gantry_offset_loc');
scorpion_gantry_offset_rot  = rospy.get_param('scorpion_gantry_offset_rot');
md_gantry_offset_loc        = rospy.get_param('md_gantry_offset_loc');

### ---------------------------- Preset parameters --------------------------------- ###
# TODO find out the values
scorpion_gantry_offset_loc  = [];
scorpion_gantry_offset_rot  = [];
probe_base_offset_loc       = rospy.get_param('probe_base_offset_loc');
probe_base_offset_rot       = rospy.get_param('probe_base_offset_rot');
rate                        = 50;
gantry_width                = rospy.get_param('gantry_width')
gantry_sweep_speed          = rospy.get_param('gantry_sweep_speed')
gantry_trans_speed          = gantry_sweep_speed;
gantry_rot_speed            = rospy.get_param('gantry_rot_speed');     #rad per second

### ---------------------------- Parameters that are updated ------------------------ ###

current_state               = 0;
gantry_mode                 = 0;
# gantry_cmd                  = [0]*6;
gantry_cmd                  = to_gantry_msg();
# int16 state_desired
# float64 sweep_speed_desired       mm/s
# float64 x_desired                 mm
# float64 y_desired                 mm
# float64 yaw_desired               rad
# float64 probe_angle_desired       rad
sensor_head                 = [0]*6;
probe_yaw_angle             = probe_base_offset_rot[2];
vel_dir                     = 1;
desired_state_reached       = False;

### --------------------------- Updating the modes and commands -------------------- ###

def update_gantry_mode(data):
    global gantry_mode;
    gantry_mode = data.data;

def update_gantry_cmd(data):
    global gantry_cmd;
    global gantry_state;
    gantry_cmd          = data;
    # gantry_state        = gantry_cmd[0];

# int16 state_desired
# float64 sweep_speed_desired       mm/s
# float64 x_desired                 mm
# float64 y_desired                 mm
# float64 yaw_desired               rad
# float64 probe_angle_desired       rad

### --------------------------------- Publish Transforms --------------------------- ###

def publish_transforms():
    global br;
    global scorpion_gantry_offset_loc;
    global scorpion_gantry_offset_rot;
    global probe_base_offset_loc;
    global probe_base_offset_rot;
    global sensor_head;
    global probe_yaw_angle;

    # static
    #br.sendTransform((scorpion_gantry_offset_loc[0],
    #                scorpion_gantry_offset_loc[1],
    #                scorpion_gantry_offset_loc[2]),
    #                tf.transformations.quaternion_from_euler(scorpion_gantry_offset_rot[0],
    #                                                        scorpion_gantry_offset_rot[1],
    #                                                        scorpion_gantry_offset_rot[2]),
    #                rospy.Time.now(),
    #                "gantry",
    #                "scorpion")

    # sent by teensy/sim
    br.sendTransform((sensor_head[0],sensor_head[1],sensor_head[2]),
        tf.transformations.quaternion_from_euler(sensor_head[3],sensor_head[4],sensor_head[5]),
        rospy.Time.now(),
        "sensor_head",
        "gantry")

    # sent by teensy/sim
    br.sendTransform((md_gantry_offset_loc[0],
        md_gantry_offset_loc[1],
        md_gantry_offset_loc[2]),
        tf.transformations.quaternion_from_euler(0,0,0),
        rospy.Time.now(),
        "md",
        "sensor_head")

    # sent by teensy/sim
    br.sendTransform((probe_base_offset_loc[0],
        probe_base_offset_loc[1],
        probe_base_offset_loc[2]),
        tf.transformations.quaternion_from_euler(probe_base_offset_rot[0],
            probe_base_offset_rot[1],
            probe_yaw_angle),
        rospy.Time.now(),
        "probe_base",
        "sensor_head")

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
    lat_vel             = gantry_sweep_speed;

    #normally we will check if they are out of origin
    #move them slowly back, before we start sweeping
    #for simplicity we will just zero them instantly
    sensor_head[0] = 0;
    sensor_head[2] = 0;
    sensor_head[3] = 0;
    sensor_head[4] = 0;
    sensor_head[5] = 0;

    sensor_head[1] += vel_dir * lat_vel / rate;
    if sensor_head[1] < low_lim + tolerance or sensor_head[1] > high_lim - tolerance:
        vel_dir *= -1;

    #sensor_head[0] = trans[0]
    #sensor_head[1] = trans[1]
    #sensor_head[2] = trans[2]
    #sensor_head[3] = rot[0]
    #sensor_head[4] = rot[1]
    #sensor_head[5] = rot[2]

def actuate_to_desired():
    global sensor_head;
    global probe_yaw_angle;         #probe base to sensor_head
    global rate;
    global gantry_cmd;
    global sensor_head;
    global gantry_trans_speed;
    global gantry_rot_speed;
    global desired_state_reached;

    lat_vel = gantry_sweep_speed;
    diff    = np.zeros(4);

    trans_mm_per_loop   = gantry_trans_speed / float(rate);
    rot_rad_per_loop    = gantry_rot_speed / float(rate);

    #assuming it updates everytime this runs
    #otherwise we will offset twice
    # gantry_cmd[2] -= md_gantry_offset_loc[0]
    # gantry_cmd[3] -= md_gantry_offset_loc[1]
    # temp_gantry_cmd = [gantry_cmd[0],
    #                     gantry_cmd[1],
    #                     gantry_cmd[2],    # x
    #                     gantry_cmd[3],    # y
    #                     gantry_cmd[4],
    #                     gantry_cmd[5]];

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

    # probe yaw
    probe_yaw_diff  = gantry_cmd.probe_angle_desired - probe_yaw_angle;
    probe_yaw_dir   = np.sign(probe_yaw_diff);
    if (abs(probe_yaw_diff) <= rot_rad_per_loop):
        probe_yaw_angle = gantry_cmd.probe_angle_desired;
        desired_state_reached_flag += 1;
    else:
        probe_yaw_angle+= probe_yaw_dir * rot_rad_per_loop;

    if (desired_state_reached_flag == 4):
        desired_state_reached   = True;
    else:
        desired_state_reached   = False;

### ----------------------------------------------------------------------------- ###

def main():
    global gantry_mode;
    global gantry_cmd;
    global sensor_head;
    global probe_yaw_angle;
    global desired_state_reached;
    global current_state;

    rospy.init_node('gantry_sim')

    global br
    br = tf.TransformBroadcaster()

    # update mode
    gantry_mode_sub         = rospy.Subscriber("gantry_cmd_hack_send", Int16, update_gantry_mode);
    gantry_cmd_sub          = rospy.Subscriber("gantry_cmd_send", to_gantry_msg, update_gantry_cmd);
    # from gantry
    gantry_current_state_pub= rospy.Publisher("/gantry_current_state", gantry_status, queue_size=1);
    #to gantry teensy/sim

    rate    = 50;
    r       = rospy.Rate(rate);

    while not rospy.is_shutdown():
        ### idle ###
        if gantry_mode == 0:
            pass;

        ### calibrate ###
        elif gantry_mode == 1:
            pass;

        ### sweeping ###
        elif gantry_mode == 2:
            sweep();

        ### moving to position ###
        elif gantry_mode == 3 or gantry_mode == 4:
            actuate_to_desired();

        # gantry teensy only publishes transforms
        publish_transforms();

        ### ------------- PREPARE MESSAGES TO PUBLISH ---------------- ###

        #prepare the messages


        gantry_current_state = gantry_status()
        gantry_current_state.state = current_state
        gantry_current_state.sweep_speed = 0 # TODO
        gantry_current_state.x = sensor_head[0]
        gantry_current_state.y = sensor_head[1]
        gantry_current_state.yaw = sensor_head[5]            # rad
        gantry_current_state.probe_angle = probe_yaw_angle   # rad
        gantry_current_state.position_reached = desired_state_reached

        # publish the messages
        gantry_current_state_pub.publish(gantry_current_state);

        r.sleep();

if __name__ == "__main__":
    main()

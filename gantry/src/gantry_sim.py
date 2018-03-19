#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
import tf

# scorpion_gantry_offset_loc = rospy.get_param('scorpion_gantry_offset_loc')
# scorpion_gantry_offset_rot = rospy.get_param('scorpion_gantry_offset_rot')
# md_gantry_offset_loc = rospy.get_param('md_gantry_offset_loc')
# probe_base_offset_loc = rospy.get_param('probe_base_offset_loc')
# probe_base_offset_rot = rospy.get_param('probe_base_offset_rot')

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
rate                        = 100;
gantry_width                = rospy.get_param('gantry_width')
gantry_sweep_speed          = rospy.get_param('gantry_sweep_speed')

### ---------------------------- Parameters that are updated ------------------------ ###

gantry_mode                 = 0;
gantry_cmd                  = [0]*6;
sensor_head                 = [0]*6;
probe_yaw_angle             = probe_base_offset_rot[2];
vel_dir                     = 1;

### --------------------------- Updating the modes and commands -------------------- ###

def update_gantry_mode(data):
    global gantry_mode;
    gantry_mode = data.data;

def update_gantry_cmd(data):
    global gantry_cmd;
    global gantry_state;
    global sweep_speed;
    gantry_cmd          = data.data;
    gantry_state        = gantry_cmd[0];
    gantry_sweep_speed  = gantry_cmd[1];

'''
    Int16MultiArray_msg     = [current_state,
                                sweep_velocity,
                                x_position,
                                y_position,
                                yaw,
                                probe_yaw,
                                int(desired_state_reached)];
'''

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

    #continue sweeping
    # convert desired md position to desired sensor_head position

    #(trans,rot) = listener.lookupTransform('/gantry', '/sensor_head', rospy.Time(0))
    #trans[1] += vel_dir * lat_vel / rate
    #if trans[1] < low_lim + tolerance or trans[1] > high_lim - tolerance:
    #    vel_dir *= -1

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
    global probe_yaw_angle;
    global gantry_mode;
    global rate;
    global gantry_cmd;
    global sensor_head;
    global gantry_width;
    global gantry_sweep_speed;
    global vel_dir;

    lat_vel = gantry_sweep_speed;
    diff    = np.zeros(4);

    #assuming it updates everytime this runs
    #otherwise we will offset twice
    # gantry_cmd[2] -= md_gantry_offset_loc[0]
    # gantry_cmd[3] -= md_gantry_offset_loc[1]
    temp_gantry_cmd = [gantry_cmd[0],
                        gantry_cmd[1],
                        gantry_cmd[2] - md_gantry_offset_loc[0],
                        gantry_cmd[3] - md_gantry_offset_loc[1],
                        gantry_cmd[4],
                        gantry_cmd[5]];

    #diff = [gantry_cmd[i] - sensor_head[i] for i in range(4)]
    #for i in range(4):
    #    if abs(diff[i]) < (lat_vel / rate):
    #        sensor_head[i] = cmd[i]
    #    else:
    #        if diff[i] > 0:
    #            sensor_head[i] += (lat_vel / rate)
    #        else:
    #            sensor_head[i] -= (lat_vel / rate)

    #sensor_head[0] = trans[0]
    #sensor_head[1] = trans[1]
    #sensor_head[2] = trans[2]
    #sensor_head[3] = rot[0]
    #sensor_head[4] = rot[1]
    #sensor_head[5] = rot[2]

    # X Position
    diff[0] = (temp_gantry_cmd[2] - sensor_head[0])/rate;
    sensor_head[0] += diff[0];

    # Y Position
    diff[1] = (temp_gantry_cmd[3] - sensor_head[1])/rate;
    sensor_head[1] += diff[1];

    # Gantry Yaw
    diff[2] = (temp_gantry_cmd[4] - sensor_head[5])/rate;
    sensor_head[5] += diff[2];

    # Probe Yaw
    diff[3] = (temp_gantry_cmd[5] - probe_yaw_angle)/rate;
    probe_yaw_angle += diff[3];

    if abs(np.sum(diff)) < 0.1:
        desired_state_reached = True;
    else:
        desired_state_reached = False;

    #diff = [gantry_cmd[i] - sensor_head[i] for i in range(4)]
    #for i in range(4):
    #    if abs(diff[i]) < (lat_vel / rate):
    #        sensor_head[i] = cmd[i]
    #    else:
    #        if diff[i] > 0:
    #            sensor_head[i] += (lat_vel / rate)
    #        else:
    #            sensor_head[i] -= (lat_vel / rate)

### ----------------------------------------------------------------------------- ###

def main():
    global gantry_mode;
    global gantry_cmd;

    rospy.init_node('gantry_sim')

    global br
    br = tf.TransformBroadcaster()

    # update mode
    gantry_mode_sub     = rospy.Subscriber("gantry_cmd_hack_send", Int16, update_gantry_mode);
    gantry_cmd_sub      = rospy.Subscriber("gantry_cmd_send", Int16MultiArray, update_gantry_cmd);

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
        elif gantry_mode == 3:
            actuate_to_desired();
            
        # gantry teensy only publishes transforms
        publish_transforms();

        r.sleep();

if __name__ == "__main__":
    main()

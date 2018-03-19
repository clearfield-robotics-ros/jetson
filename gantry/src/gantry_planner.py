#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
import tf

from gantry_lib_for_sim_WIP import Gantry

# in: command of sweeping / position
# out: position of gantry (geometry_msgs/Point)

#JETSON
current_state               = 0;            # status of jetson
desired_state               = 0;            # what we want the jetson to be in

#GANTRY
# cmd                         = [0, 0, 0];    # commands from metal detector
cmd                         = Int16MultiArray();
sensor_head                 = [0]*6;
desired_state_reached       = False;

### ------------------ TRANSFORMS ---------------------------- ###

#setup params
scorpion_gantry_offset_loc  = rospy.get_param('scorpion_gantry_offset_loc');
scorpion_gantry_offset_rot  = rospy.get_param('scorpion_gantry_offset_rot');
md_gantry_offset_loc        = rospy.get_param('md_gantry_offset_loc');
probe_base_offset_loc       = rospy.get_param('probe_base_offset_loc');
probe_base_offset_rot       = rospy.get_param('probe_base_offset_rot');
probe_yaw_angle             = probe_base_offset_rot[2];

### -------------------- monitor current state --------------- ###
def update_state(data):
    global current_state;
    current_state           = data.data;

### ---------------------------------------------------------- ###
def update_md_cmd(data):
    global cmd;
    global current_state;
    global probe_yaw_angle;
    cmd                     = [current_state, 
                                0,              # NOT USED
                                data.x,         # desired x             MM
                                data.y,         # desired y             MM
                                data.z,         # desired yaw           DEG
                                probe_yaw_angle,# desired probe yaw     DEG
                                0];              # not used)

def update_gantry_desired_state(data):
    global gantry_desired_state
    gantry_desired_state    = data.data

'''
    gantry_desired_state    = [current_state,
                                sweep_velocity,
                                x_position,
                                y_position,
                                yaw,
                                probe_yaw,
                                int(desired_state_reached)];
'''


#getting the transforms out there
def publish_transforms():
    global br

    # static
    br.sendTransform((scorpion_gantry_offset_loc[0],
        scorpion_gantry_offset_loc[1],
        scorpion_gantry_offset_loc[2]),
        tf.transformations.quaternion_from_euler(scorpion_gantry_offset_rot[0],
            scorpion_gantry_offset_rot[1],
            scorpion_gantry_offset_rot[2]),
        rospy.Time.now(),
        "gantry",
        "scorpion")

    # sent by teensy/sim
    #br.sendTransform((sensor_head[0],sensor_head[1],sensor_head[2]),
    #    tf.transformations.quaternion_from_euler(sensor_head[3],sensor_head[4],sensor_head[5]),
    #    rospy.Time.now(),
    #    "sensor_head",
    #    "gantry")

    # sent by teensy/sim
    #br.sendTransform((md_gantry_offset_loc[0],
    #    md_gantry_offset_loc[1],
    #    md_gantry_offset_loc[2]),
    #    tf.transformations.quaternion_from_euler(0,0,0),
    #    rospy.Time.now(),
    #    "md",
    #    "sensor_head")

    # sent by teensy/sim
    #br.sendTransform((probe_base_offset_loc[0],
    #    probe_base_offset_loc[1],
    #    probe_base_offset_loc[2]),
    #    tf.transformations.quaternion_from_euler(probe_base_offset_rot[0],
    #        probe_base_offset_rot[1],
    #        probe_yaw_angle),
    #    rospy.Time.now(),
    #    "probe_base",
    #    "sensor_head")


def main():
    global current_state;
    global cmd;
    global sensor_head;
    global desired_state_reached;
    global probe_base_offset_rot;
    global probe_yaw_angle;
    probe_yaw_angle = probe_base_offset_rot[2];

    rospy.init_node('gantry_planner');

    # broadcaster and listener
    global br
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()

    #gantry teensy/sim object
    g = Gantry();

    # from jetson
    jetson_current_state    = rospy.Subscriber('current_state', Int16, update_state);
    # from metal detector
    md_subscriber           = rospy.Subscriber("cmd_from_md", Point, update_md_cmd);
    # from probe
    gantry_desired_state    = rospy.Subscriber("/gantry_desired_state", Int16MultiArray, update_gantry_desired_state);
    
    # from gantry
    gantry_current_state_pub= rospy.Publisher("/gantry_current_state", Int16MultiArray, queue_size=1);
    #to gantry teensy/sim
        #use g.send_state
        #use g.send_pos_cmd

    rate = 50
    r = rospy.Rate(rate)  # 100 Hz

    while not rospy.is_shutdown():
        publish_transforms();
        ### ----------------- UPDATE GANTRY POSITION ----------------- ###
        try:
            (trans,rot)     = listener.lookupTransform('/gantry', '/sensor_head', rospy.Time(0))
            #all this will be handled by gantry_sim
            # trans[1] += vel_dir * lat_vel / rate
            # if trans[1] < low_lim + tolerance or trans[1] > high_lim - tolerance:
            #     vel_dir *= -1
            sensor_head[0]  = trans[0]
            sensor_head[1]  = trans[1]
            sensor_head[2]  = trans[2]
            sensor_head[3]  = rot[0]
            sensor_head[4]  = rot[1]
            sensor_head[5]  = rot[2]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


        ### ----------------- STATE CHANGING LOGIC ------------------- ###
        
        # sweeping, the cmd changed to positioning
        # check current state, publish desired, jetson will switch
        # if (current_state == 2 and cmd[0] < 0):
        #     request_state_change(3);


        ### ----------------- STATE OPERATION LOGIC ------------------ ###
        
        # idle
        if current_state    == 0:
            pass;

        # calibration
        elif current_state  == 1:
            g.send_state(1);
            print ("Calibrating!");

        # sweeping
        elif current_state  == 2:
            #continue sweeping
            g.send_state(2);
            print ("Sweeping!");

        # pin pointing, listening to MD
        elif current_state  == 3:
            g.send_state(3);
            g.send_pos_cmd(cmd);
            print "positioning at " + str(cmd);

        # probing, listening to PROBE
        elif current_state  == 4:
            g.send_state(3);
            g.send_pos_cmd(cmd);
            print "Probing at " + str(cmd);

            # telling probe whether desired position is reached
            diff = np.zeros(4)
            # X Position
            diff[0]             = gantry_desired_state[2] - sensor_head[0];
            # sensor_head[0] += diff[0]
            # Y Position
            diff[1]             = gantry_desired_state[3] - sensor_head[1];
            # sensor_head[1] += diff[1]
            # Gantry Yaw
            diff[2]             = gantry_desired_state[4] - sensor_head[5];
            # sensor_head[5] += diff[2]
            # Probe Yaw
            diff[3]             = gantry_desired_state[5] - probe_yaw_angle;
            # probe_yaw_angle += diff[3]

            if abs(np.sum(diff)) < 1:               #THIS SHOULD BE DETERMINED BY THE TEENSY, NOT PLANNER
                desired_state_reached = True
            else:
                desired_state_reached = False


        ### ------------- PREPARE MESSAGES TO PUBLISH ---------------- ###

        #prepare the messages
        global desired_state;
        gantry_current_state_msg = Int16MultiArray();
        gantry_current_state_msg.data = [
            current_state,
            0, #current_sweep_velocity
            sensor_head[0], #current_x_position
            sensor_head[1], # current_y_position
            sensor_head[5], # current_yaw_angle
            probe_yaw_angle, #current_probe_yaw_angle
            int(desired_state_reached)];

        # publish the messages
        gantry_current_state_pub.publish(gantry_current_state_msg);


        # done
        r.sleep()  # indent less when going back to regular gantry_lib


if __name__ == "__main__":
    main()

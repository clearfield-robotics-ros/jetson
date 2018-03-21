#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
import tf

from gantry_lib_for_sim import Gantry

current_state			= 0;
desired_state 			= 0;

cmd 					= [0, 0, 0];
sensor_head 			= [0]*6;
desired_state_reached	= False;

def update_gantry_desired_state(data):
    global state;
    state 				= data[0];
    global gantry_desired_state;
    gantry_desired_state    = data.data;


### --------------------- ERRORS ------------------------ ###
def error_illegal_state_change():
	global current_state;
	current_stae 		= 0;
	print ("Error: Gantry illegal state change attempted, switching to Idle.");



### ----------------------------------------------------- ###

def main():
    rospy.init_node('gantry_planner');

    gantry_desired_state_sub= rospy.Subscriber("/gantry_desired_state", Int16MultiArray, update_gantry_desired_state);
    gantry_current_state_pub= rospy.Publisher("/gantry_current_state", Int16MultiArray);

    rate = 50;
    r = rospy.Rate(rate);

    while not rospy.is_shutdown():
    	#state machine
    	# idle
        if current_state == 0:
            if desired_state == 0:
            	pass;
            elif desired_state == 1:
            	#shift to calibration from idle
            	print ("Calibration started.");
            	current_state = 1;
            else:
            	error_illegal_state_change();
            	current_state = 0;

        # calibration
        elif current_state == 1:
        	if desired_state == 1:
        		print ("Calibrating...");
        		#do calibration, or let the jetson do it
        	elif desired_state == 2:
        		#shift to sweeping from calibration
        		print ("Sweeping started.");
        		current_state = 2;
        	else:
        		error_illegal_state_change();
        		current_state = 0;

        # sweeping
        elif current_state == 2:
        	if desired_state == 2:
        		print ("Sweeping...");
        		#do sweeping, or let the jetson do it
        	elif desired_state == 3:
        		#shift to pin pointing from sweeping
        		print ("Pin pointing started.");
        		current_state = 3;
        	else:
        		error_illegal_state_change();
        		current_state = 0;

        # pin pointing
        elif current_state == 3:
        	if desired_state == 3:
        		print







    	#updating the transforms
    	try:
            (trans,rot)     = listener.lookupTransform('/gantry', '/sensor_head', rospy.Time(0));
            sensor_head[0]  = trans[0];
            sensor_head[1]  = trans[1];
            sensor_head[2]  = trans[2];
            sensor_head[3]  = rot[0];
            sensor_head[4]  = rot[1];
            sensor_head[5]  = rot[2];

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue;

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

        #publish the messages
        gantry_current_state_pub.publish(gantry_current_state_msg);
    	r.sleep();



if __name__ == "__main__":
    main();

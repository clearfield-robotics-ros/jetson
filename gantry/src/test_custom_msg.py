#!/usr/bin/env python

import rospy;
from gantry.msg import gantry_status;
from gantry.msg import to_gantry_msg;

def main():
	rospy.init_node("test_custom_msg");
	pub_status 		= rospy.Publisher("states", gantry_status, queue_size=10);
	pub_to_gantry 	= rospy.Publisher("desired", to_gantry_msg, queue_size=10);

	rate 			= 20;
	r 				= rospy.Rate(rate);

	while not rospy.is_shutdown():
		state 		= gantry_status();
		state.state  	 		= 1;
		state.sweep_speed 		= 1;
		state.x 				= 1;
		state.y 				= 2.13;
		state.yaw 				= 3.214;
		state.probe_angle 		= 4.222;
		state.position_reached 	= False;

		msg 		= to_gantry_msg();
		msg.state_desired 	 	= 2;
		msg.sweep_speed_desired	= 2;
		msg.x_desired 			= 2;
		msg.y_desired 			= 2;
		msg.yaw_desired 		= 2;
		msg.probe_angle_desired = 324.21;

		pub_status.publish(state);
		pub_to_gantry.publish(msg);

		r.sleep();

if __name__ == "__main__":
	main();
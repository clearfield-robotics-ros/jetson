#!/usr/bin/env python

import rospy
import numpy as np
import tf
import math
import pdb
from mine_estimator import Mine_Estimator
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from probe.msg import probe_data
from gantry.msg import gantry_status
from gantry.msg import to_gantry_msg;
from constraints import Probe_Motion_Planner
from shapely.geometry import Point as shapely_Point
import time


def main():
	rospy.init_node('mine_estimator_tester')

	landmine_diameter   = rospy.get_param('landmine_diameter')
	landmine_height     = rospy.get_param('landmine_height')

	test_data			= rospy.get_param('test_data')

	est_mine_list = []
	est_mine_list.append(Mine_Estimator(landmine_diameter, landmine_height))

	rospy.sleep(0.5)
	r = rospy.Rate(10) # Hz
	while not rospy.is_shutdown():

		for test in range(0,len(test_data)):

			print "mine #",test+1
			est_mine_list[-1].clear_markers()
			est_mine_list.append(Mine_Estimator(landmine_diameter, landmine_height))

			truth = test_data[test]['class']
			points = test_data[test]['points']

			for j in range(0,len(points)):
				est_mine_list[-1].add_point(points[j][0],points[j][1],points[j][2])

			est_mine_list[-1].circle_fit()
			est_mine_list[-1].print_results()

			if est_mine_list[-1].get_result() == truth:
				print "Correct!"
			else:
				print "Incorrect :("

			raw_input('...')
		return
	r.sleep()

if __name__ == "__main__":
    main()

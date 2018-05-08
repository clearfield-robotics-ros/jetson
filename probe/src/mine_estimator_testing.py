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

class Results:
	def __init__(self):
		self.correct = []
		self.attempted_probes = []
		self.inliers = []
		self.error = []

	def count(self):
		return len(self.correct)

	def accuracy(self):
		return (float(self.correct.count(True))/float(len(self.correct))*100)

	def mean_attempted_probes(self):
		return self.mean(self.attempted_probes)

	def mean_inliers(self):
		return self.mean(self.inliers)

	def mean_metric(self):
		return self.mean_inliers()/self.mean_attempted_probes()*100

	def mean_error(self):
		return self.mean(self.error)

	def mean(self, l):
		return sum(l) / float(len(l))

def main():
	rospy.init_node('mine_estimator_tester')

	true = Results()
	false = Results()

	landmine_diameter   = rospy.get_param('landmine_diameter')
	landmine_height     = rospy.get_param('landmine_height')
	test_data			= rospy.get_param('test_data')

	est_mine_list = []
	est_mine_list.append(Mine_Estimator(landmine_diameter, landmine_height))

	rospy.sleep(0.5)
	r = rospy.Rate(10) # Hz
	while not rospy.is_shutdown():

		for test in range(0,len(test_data)):

			# Setup Test Data
			print "mine #",test+1
			est_mine_list[-1].clear_markers()
			est_mine_list.append(Mine_Estimator(landmine_diameter, landmine_height))

			truth = test_data[test]['class']
			attempted = test_data[test]['attempted']
			points = test_data[test]['points']

			# Add Points
			for j in range(0,len(points)):
				est_mine_list[-1].add_point(points[j][0],points[j][1],points[j][2])

			# Compute Results
			est_mine_list[-1].set_probe_attempts(attempted)
			est_mine_list[-1].circle_fit()
			est_mine_list[-1].print_results(True)

			# Append Results
			if truth == True:

				true.error.append(est_mine_list[-1].error)
				true.inliers.append(est_mine_list[-1].num_inliers)
				true.attempted_probes.append(est_mine_list[-1].num_attempted_probes)

				if est_mine_list[-1].get_result() == truth:
					print "Correct!"
					true.correct.append(True)
				else:
					print "Incorrect :("
					true.correct.append(False)
			else:

				false.error.append(est_mine_list[-1].error)
				false.inliers.append(est_mine_list[-1].num_inliers)
				false.attempted_probes.append(est_mine_list[-1].num_attempted_probes)

				if est_mine_list[-1].get_result() == truth:
					print "Correct!"
					false.correct.append(True)
				else:
					print "Incorrect :("
					false.correct.append(False)

			# raw_input('...') # pause to visualize
			rospy.sleep(0.5)

		print "\n\n\n-----------------------"
		print "Landmine Results"
		print "-----------------------"
		print "Total Landmines Surveyed:", true.count()
		print "Correctly Clasified: %0.1f%%" % true.accuracy()
		print "Mean Attempted Probes: %0.1f" % true.mean_attempted_probes()
		print "Mean Inier Points: %0.1f" % true.mean_inliers()
		print "Mean Landmine Metric: %0.1f%%" % true.mean_metric()
		print "Mean Error: %0.1fmm" % true.mean_error()
		print "-----------------------"
		print "Non-Landmine Results"
		print "-----------------------"
		print "Total Non-Landmines Surveyed:", false.count()
		print "Correctly Clasified: %0.1f%%" % false.accuracy()
		print "Mean Attempted Probes: %0.1f" % false.mean_attempted_probes()
		print "Mean Inier Points: %0.1f" % false.mean_inliers()
		print "Mean Landmine Metric: %0.1f%%" % false.mean_metric()
		print "Mean Error: %0.1fmm" % false.mean_error()
		print "-----------------------"
		return

	r.sleep()

if __name__ == "__main__":
    main()

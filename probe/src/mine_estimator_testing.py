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

def polar_to_cartesian(r, th):
	x_offset = 0#500
	y_offset = 0#500
	x = -math.cos(th) * r + x_offset# + 2*np.random.random()-1
	y = -math.sin(th) * r + y_offset# + 2*np.random.random()-1

	return x, y

def main():
	rospy.init_node('mine_loc_tester')
	# br = tf.TransformBroadcaster()


	landmine_diameter               = rospy.get_param('landmine_diameter')
	landmine_height                 = rospy.get_param('landmine_height')

	est_mine_list = []
	est_mine_list.append(Mine_Estimator(landmine_diameter, landmine_height))

	p = []

	# 1
	p.append( np.array([[ 329.41020804, 268.3913269, -466.07662089], \
	[ 335.87816301, 309.2696103, -464.13026711], \
	[ 344.70618721, 231.81042454, -469.84472658]]) )

	# 2
	p.append( np.array([[ 392.32877649, 529.02527847, -490.02722396], \
	[ 433.48694835, 531.89043748, -488.8082696 ]]) )

	# 2
	p.append( np.array([[0,0,0]]) )

	# 4
	p.append( np.array([[ 355.56676951, 273.38131714, -470.44243296],
	[ 366.49356866, 308.52185559, -471.91134106],
	[ 372.55872946, 241.67211997, -475.8373742 ]]) )

	# 5
	p.append( np.array([[ 413.20804771, 324.32476748, -492.23711708],
	[ 407.94407242, 283.2135808, -488.82969853]]) )

	#6
	p.append( np.array([[ 370.18690239, 572.17277946, -481.38525435],
	[ 377.90913947, 592.85471573, -490.96549687],
	[ 387.31570841, 575.20623563, -490.1136379 ]]) )

	#7
	p.append( np.array([[ 303.48580921, 247.0055542, -483.27384307],
	[ 335.22170744, 276.57796644, -493.96553515],
	[ 313.80894584, 193.32290511, -475.3482407 ],
	[ 300.42235358, 277.96524464, -477.24945592],
	[ 297.59746946, 214.79193563, -475.42088302]]) )

	#8
	p.append( np.array([[ 402.65616433, 312.94497681, -488.48403545],
	[ 420.70807834, 328.63696253, -496.74329093],
	[ 413.60473593, 293.47356896, -492.14523349]]) )

	#9
	p.append( np.array([[ 329.44500212, 252.18970197, -496.0519302 ],
	[ 293.49367869, 201.67452371, -472.78031517]]) )

	#10
	p.append( np.array([[ 416.73804313, 555.76978196, -493.68158928],
	[ 387.22070214, 563.61938467, -479.27415263],
	[ 383.3362271, 551.51820807, -475.18772661]]) )

	#11
	p.append( np.array([[ 407.24786002, 150.06564138, -497.12599448]] ) )

	#12
	p.append( np.array([[ 367.34344218, 331.83575439, -467.99019967],
	[ 377.94765482, 369.29313473, -468.95315387],
	[ 380.71314034, 295.7803806, -470.74327752]]) )

	r = rospy.Rate(10) # Hz
	while not rospy.is_shutdown():


		for i in range(0,12):
			print "mine #",i+1
			raw_input('...')

			for j in range(0,len(p[i])):
				est_mine_list[-1].add_point(p[i][j][0],p[i][j][1],p[i][j][2])

			est_mine_list[-1].print_results()
			est_mine_list.append(Mine_Estimator(landmine_diameter, landmine_height))

		return

	r.sleep()

if __name__ == "__main__":
    main()

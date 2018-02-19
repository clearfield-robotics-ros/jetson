#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PointStamped, Point
import numpy as np
import math


def setTarget(data):
	print "We're in probing mode!"
	print data





def main():
    # global pub
    rospy.init_node('searching')
	
    # get mine diameter from config file

	# take md centre input
	sub = rospy.Subscriber('/MDToProbe', Point, setTarget)

    rospy.spin()

    #######################
	# begin probing sequence, as in matlab / old c++ code
	#######################

if __name__ == "__main__":
    main()

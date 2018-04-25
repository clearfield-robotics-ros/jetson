#!/usr/bin/env python

import rospy
from gantry.msg import detection_result
import math
import random

def main():
    rospy.init_node('probe_sim')

    pub = rospy.Publisher("/results", detection_result, queue_size=10)

    id = 1
    for i in range(1,6): # do 10 times

        detection = detection_result()

        detection.id = id
        detection.truth = random.choice([True, False])
        detection.radius_truth = .500
        detection.x_truth = random.uniform(0, 1.000)
        detection.y_truth = random.uniform(0, 1.000)

        detection.estimate = random.choice([True, False])
        detection.radius_estimate = random.uniform(1, 1.000)
        detection.x_estimate = random.uniform(0, 1.000)
        detection.y_estimate = random.uniform(0, 1.000)

        detection.estimate_euclidean_error = math.sqrt((detection.x_truth - detection.x_estimate)**2
            + (detection.y_truth - detection.y_estimate)**2)

        detection.warning_delay = random.uniform(0, 1000)
        detection.probe_time = random.uniform(0, 5)

        pub.publish(detection)
        print detection
        id += 1

        rospy.sleep(0.5)

    rospy.spin()

    # r = rospy.Rate(100)  # 100 Hz

    # while not rospy.is_shutdown():

        # print "running"

    # r.sleep()  # indent less when going back to regular gantry_lib

if __name__ == "__main__":
    main()

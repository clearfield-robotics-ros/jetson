#!/usr/bin/env python

import rospy
import tf
from sensor_msgs.msg import NavSatFix

def get_gps_coords(index):
    gps_coords = [[40.4731862394, -79.9654294237],
                  [40.4724282625, -79.96543305],
                  [40.4725429322, -79.9654184004]]
    return gps_coords[index]


def main():
    rospy.init_node('mine_loc_tester')

    # Transforms
    global listener
    listener = tf.TransformListener()


    # Probe Related Messages
    mine_loc_pub = rospy.Publisher("/gps/mine_loc", NavSatFix, queue_size=10)


    r = rospy.Rate(0.5) # Hz
    while not rospy.is_shutdown():
        for i in range(3):
            gps_coords = get_gps_coords(i)
            mine_loc_pub.publish(NavSatFix(latitude = gps_coords[0], longitude = gps_coords[1]))
            r.sleep()


if __name__ == "__main__":
    main()

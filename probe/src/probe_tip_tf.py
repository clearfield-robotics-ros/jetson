#!/usr/bin/env python

import rospy
import tf

# THIS IS BASICALLY A PARAMETERIZED ROS STATIC TF PUBLISHER

def main():
    rospy.init_node('probe_tip_tf')

    br = tf.TransformBroadcaster()

    probe_offset_distance = rospy.get_param('probe_offset_distance')

    r = rospy.Rate(100) # Hz
    while not rospy.is_shutdown():

        br.sendTransform((probe_offset_distance,0,0),
            tf.transformations.quaternion_from_euler(0,0,0),
            rospy.Time.now(),
            "probe_tip",
            "probe_car")

        r.sleep()

if __name__ == "__main__":
    main()

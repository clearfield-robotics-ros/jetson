#!/usr/bin/env python

import rospy
import tf

# THIS IS BASICALLY A PARAMETERIZED ROS STATIC TF PUBLISHER

#getting the transforms out there

def main():
    rospy.init_node('gantry_tf')

    br = tf.TransformBroadcaster()

    scorpion_gantry_offset_loc  = rospy.get_param('scorpion_gantry_offset_loc');#mm
    scorpion_gantry_offset_rot  = rospy.get_param('scorpion_gantry_offset_rot');#rad

    r = rospy.Rate(100) # Hz
    while not rospy.is_shutdown():

        br.sendTransform((scorpion_gantry_offset_loc[0],
                          scorpion_gantry_offset_loc[1],
                          scorpion_gantry_offset_loc[2]),
                          tf.transformations.quaternion_from_euler(scorpion_gantry_offset_rot[0],
                                                                    scorpion_gantry_offset_rot[1],
                                                                    scorpion_gantry_offset_rot[2]),
                          rospy.Time.now(),
                          "gantry",
                          "scorpion");

        r.sleep()

if __name__ == "__main__":
    main()

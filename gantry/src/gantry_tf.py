#!/usr/bin/env python

import rospy
import tf

# THIS IS BASICALLY A PARAMETERIZED ROS STATIC TF PUBLISHER

#getting the transforms out there

def main():
    rospy.init_node('gantry_tf')

    br = tf.TransformBroadcaster()

    scorpion_gantry_offset_loc      = rospy.get_param('scorpion_gantry_offset_loc');#mm
    scorpion_gantry_offset_rot      = rospy.get_param('scorpion_gantry_offset_rot');#rad

    sensorhead_md_offset_loc        = rospy.get_param('sensorhead_md_offset_loc');
    sensorhead_md_offset_rot        = rospy.get_param('sensorhead_md_offset_rot');

    sensorhead_probebase_offset_loc = rospy.get_param('sensorhead_probebase_offset_loc');
    sensorhead_probebase_offset_rot = rospy.get_param('sensorhead_probebase_offset_rot');

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
            "scorpion")


        # sent by teensy/sim
        br.sendTransform((sensorhead_md_offset_loc[0],
                        sensorhead_md_offset_loc[1],
                        sensorhead_md_offset_loc[2]),
            tf.transformations.quaternion_from_euler(sensorhead_md_offset_rot[0],
                                                    sensorhead_md_offset_rot[1],
                                                    sensorhead_md_offset_rot[2]),
            rospy.Time.now(),
            "md",
            "sensor_head")


        # sent by teensy/sim
        br.sendTransform((sensorhead_probebase_offset_loc[0],
                          sensorhead_probebase_offset_loc[1],
                          sensorhead_probebase_offset_loc[2]),
            tf.transformations.quaternion_from_euler(sensorhead_probebase_offset_rot[0],
                                                    sensorhead_probebase_offset_rot[1],
                                                    sensorhead_probebase_offset_rot[2]),
           rospy.Time.now(),
            "probe_base",
            "sensor_head")

        r.sleep()

if __name__ == "__main__":
    main()
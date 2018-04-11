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
    gantry_md_offset_loc        = rospy.get_param('gantry_md_offset_loc');      #mm
    gantry_probe_offset_loc     = rospy.get_param('gantry_probe_offset_loc');   #mm
    gantry_probe_offset_rot     = rospy.get_param('gantry_probe_offset_rot');   #rad

    r = rospy.Rate(100) # Hz
    while not rospy.is_shutdown():

        scorpion_gantry_offset_rot_quat   = tf.transformations.quaternion_from_euler(scorpion_gantry_offset_rot[0],
                                                                                      scorpion_gantry_offset_rot[1],
                                                                                      scorpion_gantry_offset_rot[2]);
        br.sendTransform((scorpion_gantry_offset_loc[0],
                          scorpion_gantry_offset_loc[1],
                          scorpion_gantry_offset_loc[2]),
                          scorpion_gantry_offset_rot_quat,
                          rospy.Time.now(),
                          'gantry_zero',
                          'scorpion');

        br.sendTransform((gantry_md_offset_loc[0],
                          gantry_md_offset_loc[1],
                          gantry_md_offset_loc[2]),
                          scorpion_gantry_offset_rot_quat,
                          rospy.Time.now(),
                          'md',
                          'gantry');

        gantry_probe_offset_rot_quat      = tf.transformations.quaternion_from_euler(gantry_probe_offset_rot[0],
                                                                                      gantry_probe_offset_rot[1],
                                                                                      gantry_probe_offset_rot[2]);
        br.sendTransform((gantry_probe_offset_loc[0],
                          gantry_probe_offset_loc[1],
                          gantry_probe_offset_loc[2]),
                          gantry_probe_offset_rot_quat,
                          rospy.Time.now(),
                          'probe_base',
                          'gantry');

        r.sleep()

if __name__ == "__main__":
    main()

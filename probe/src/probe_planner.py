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

### monitor current state ###
current_state = 0 # if we don't get msgs
def update_state(data):
    global current_state
    current_state = data.data
jetson_current_state = rospy.Subscriber('current_state', Int16, update_state)


def probe_to_gantry_transform(loc,yaw):

    Hprobe = np.array([[1,0,0,loc.x],
                       [0,1,0,loc.y],
                       [0,0,1,loc.z],
                       [0,0,0,1]])

    Hyaw = np.array([[math.cos(yaw),-math.sin(yaw),0,0],
                     [math.sin(yaw),math.cos(yaw),0,0],
                     [0,0,1,0],
                     [0,0,0,1]])

    Hyrot = np.array([[math.cos(-probe_angle),0,math.cos(-probe_angle),0],
                      [0,1,0,0],
                      [math.sin(-probe_angle),0,math.cos(-probe_angle),0],
                      [0,0,0,1]])

    Hd = np.array([[1,0,0,-probe_length],
                   [0,1,0,0],
                   [0,0,1,0],
                   [0,0,0,1]])

    Hoffset = np.array([[1,0,0,-sensorhead_probebase_offset_loc[0]],
                       [0,1,0,-sensorhead_probebase_offset_loc[1]],
                       [0,0,1,-sensorhead_probebase_offset_loc[2]],
                       [0,0,0,1]])

    H = Hprobe.dot(Hyaw).dot(Hoffset).dot(Hyrot).dot(Hd)
    trans = np.matmul(H,np.array([[0],[0],[0],[1]]))

    return trans


def move_gantry(desired_probe_tip, gantry_yaw):

    print "GANTRY YAW:", gantry_yaw*180/math.pi
    print "GANTRY POSIITON:", desired_probe_tip

    # work out gantry carriage position
    trans = probe_to_gantry_transform(desired_probe_tip, gantry_yaw)

    plan = generate_probe_plan(trans, gantry_yaw) # only receives goal
    for i in range(len(plan)): # go through each step of plan
        gantry_desired_state = to_gantry_msg()
        gantry_desired_state.state_desired       = 3
        gantry_desired_state.x_desired           = trans[0]
        gantry_desired_state.y_desired           = plan[i][0] * 1000.0
        gantry_desired_state.yaw_desired         = plan[i][1]
        # gantry_desired_state.probe_angle_desired = 0
        global gantry_desired_state_pub
        gantry_desired_state_pub.publish(gantry_desired_state)

        rospy.sleep(0.5) # give time for handshake

        while not gantry_current_status.position_reached: # block while not finished
            pass

def generate_probe_plan(goal_trans, goal_rot):
    (current_trans, current_rot) = listener.lookupTransform('gantry', 'sensor_head', rospy.Time(0))
    current_rot_euler = tf.transformations.euler_from_quaternion(current_rot)[2]
    start_config = [current_trans[0], current_trans[1], current_rot_euler]
    end_config = [goal_trans[0], goal_trans[1], goal_rot]
    print "start config", start_config
    print "end config", end_config
    probe_motion_planner = Probe_Motion_Planner(start_config, end_config)
    path = probe_motion_planner.plan_path()
    plan_arrays = [[step.x, step.y] for step in path] # reconfigure into an array of arrays
    print plan_arrays
    return plan_arrays

def calc_probe_angle_range(desired_probe_tip):
    gantry_th_min                  = rospy.get_param('gantry_th_min')
    gantry_th_max                  = rospy.get_param('gantry_th_max')
    possible_probe_approach_angles = np.linspace(gantry_th_min, gantry_th_max, 19) # 10deg increments for 180deg 

    probe_motion_planner = Probe_Motion_Planner([0,0,0], [0,0,0]) # just instantiate with dummy points to be able to access functions

    collision_free_points = []
    for angle in possible_probe_approach_angles:
        trans = probe_to_gantry_transform(desired_probe_tip, angle)
        point_to_check = shapely_Point(trans[1]/1000.0, angle) # Point takes (y[mm], th[rad])
        collision_free_points.append(probe_motion_planner.point_collision_free(point_to_check))
    tog = zip(possible_probe_approach_angles, collision_free_points)
    allowable_angles = [x[0] for x in tog if x[1]==True] # only extract collision-free points
    # assumption! contiguous collision free points i.e. if the first is at 20deg, last at 160deg, then all of 20-160deg is free
    return [allowable_angles[0], allowable_angles[-1]]

def generate_probe_angle_sequence(index):
    num_probe_angles = 3
    angle_range = calc_probe_angle_range(get_desired_probe_tip(index))
    proportions = [0.5, 0.15, 0.85]#, 0.25, 0.75] # proportions between the lower and upper limits
    angle_sequence = [p*(angle_range[1]-angle_range[0])+angle_range[0] for p in proportions]
    return angle_sequence


def set_target(data):
    global target
    target = data
    print "target", target
    global probe_plan_state
    probe_plan_state = 0


def update_gantry_state(data):
    global gantry_current_status
    gantry_current_status = data


def update_probe_state(data):
    global probe_current_state
    probe_current_state = data

    global contact_block_flag
    if not data.probe_complete and data.contact_made and not contact_block_flag:

        print "NEW CONTACT POINT!"

        (trans,rot) = listener.lookupTransform('/gantry', '/probe_tip', rospy.Time(0))
        global est
        est.add_point(trans[0], trans[1], trans[2])
        contact_block_flag = True

    elif data.probe_complete:
        contact_block_flag = False


def main():
    rospy.init_node('probe_planner')

    global probe_plan_state
    probe_plan_state = -1 # initial state

    # Transforms
    global listener
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    # Parameters
    landmine_pos                    = rospy.get_param('landmine_pos')
    landmine_diameter               = rospy.get_param('landmine_diameter')
    landmine_height                 = rospy.get_param('landmine_height')
    global sensorhead_probebase_offset_loc
    sensorhead_probebase_offset_loc = rospy.get_param('sensorhead_probebase_offset_loc')
    probe_safety_factor             = rospy.get_param('probe_safety_factor')
    global probe_angle
    probe_angle                     = rospy.get_param('sensorhead_probebase_offset_rot')[1]
    scorpion_gantry_offset_loc      = rospy.get_param('scorpion_gantry_offset_loc')
    global probe_length
    probe_length                    = (scorpion_gantry_offset_loc[2] + sensorhead_probebase_offset_loc[2]
                                        + abs(landmine_pos[2])) / math.sin(probe_angle)
    maxForwardSearch                = math.cos(probe_angle)*landmine_height
    gantry_width                    = rospy.get_param('gantry_width')
    num_contact_points              = rospy.get_param('num_contact_points')
    min_fit_error                   = rospy.get_param('min_fit_error')
    max_num_probes                  = rospy.get_param('max_num_probes')

    probe_limit_exceeded            = False
    prev_point_count                = 0

    # Jetson Messages
    jetson_desired_state = rospy.Publisher('/desired_state', Int16, queue_size=10)

    # Gantry Control Messages
    global gantry_desired_state_pub
    gantry_desired_state_pub = rospy.Publisher("/cmd_from_probe", to_gantry_msg, queue_size=10)
    rospy.Subscriber("/gantry_current_status", gantry_status, update_gantry_state)

    # Probe Related Messages
    probe_cmd_pub = rospy.Publisher("/probe_teensy/probe_cmd_send", Int16, queue_size=10)
    desired_probe_tip = Point()
    gantry_yaw = 0
    probe_sequence = 0
    rospy.Subscriber("/probe_teensy/probe_status_reply", probe_data, update_probe_state)
    global contact_block_flag
    contact_block_flag = False

    # Recieving Target from Metal Detector
    rospy.Subscriber("/set_probe_target", Point, set_target)
    null_target = Point()
    global target
    target = null_target

    global est
    est = Mine_Estimator(landmine_diameter, landmine_height)

    # DEBUG
    # N = 4
    # rospy.sleep(0.5) # Sleeps for 1 sec
    # for i in range(0,N):
    #     x = -landmine_diameter/2*math.sin( (100 + 120/(N-1)*i) * math.pi/180)+250
    #     y = landmine_diameter/2*math.cos( (100 + 120/(N-1)*i) * math.pi/180)
    #
    #     est.add_point(x,y,0)
    #
    # rospy.sleep(0.5) # Sleeps for 1 sec
    # p = est.get_sparsest_point()
    #
    # pdb.set_trace()

    r = rospy.Rate(100) # Hz
    while not rospy.is_shutdown():

        # check we're in the right state and have a Target
        if current_state == 4 and not target == null_target:

            '''
            Perform Planning for Probe
            '''
            if probe_plan_state == 0:

                print "\nPLAN STATE 0"
                print "-----------------------"

                if probe_sequence == 0:

                    gantry_yaw = 0

                    desired_probe_tip.x = target.x - landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.y = target.y
                    desired_probe_tip.z = -scorpion_gantry_offset_loc[2] + landmine_pos[2]

                    br.sendTransform((desired_probe_tip.x,desired_probe_tip.y,desired_probe_tip.z),
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(),
                        "target",
                        "gantry")

                    move_gantry(desired_probe_tip, gantry_yaw)

                elif probe_sequence > 0:

                    if probe_sequence < max_num_probes:
                        desired_probe_tip.x += maxForwardSearch
                        move_gantry(desired_probe_tip, gantry_yaw)

                    else:
                        print "we took too many probes! skip this config"
                        probe_limit_exceeded = True

            elif probe_plan_state == 1:

                print "\nPLAN STATE 1"
                print "-----------------------"

                if probe_sequence == probe_sequence_prev:

                    gantry_yaw = +0.785398

                    desired_probe_tip.x = target.x - math.cos(gantry_yaw)*landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.y = target.y - math.sin(gantry_yaw)*landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.z = -scorpion_gantry_offset_loc[2] + landmine_pos[2]

                    move_gantry(desired_probe_tip, gantry_yaw)

                elif probe_sequence > probe_sequence_prev:

                    if (probe_sequence - probe_sequence_prev) < max_num_probes:
                        desired_probe_tip.x += math.cos(gantry_yaw)*maxForwardSearch
                        desired_probe_tip.y += math.sin(gantry_yaw)*maxForwardSearch
                        move_gantry(desired_probe_tip, gantry_yaw)

                    else:
                        print "we took too many probes! skip this config"
                        probe_limit_exceeded = True

            elif probe_plan_state == 2:

                print "\nPLAN STATE 2"
                print "-----------------------"

                if probe_sequence == probe_sequence_prev:

                    gantry_yaw = -0.785398

                    desired_probe_tip.x = target.x - math.cos(gantry_yaw)*landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.y = target.y - math.sin(gantry_yaw)*landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.z = -scorpion_gantry_offset_loc[2] + landmine_pos[2]

                    move_gantry(desired_probe_tip, gantry_yaw)

                elif probe_sequence > probe_sequence_prev:

                    if (probe_sequence - probe_sequence_prev) < max_num_probes:
                        desired_probe_tip.x += math.cos(gantry_yaw)*maxForwardSearch
                        desired_probe_tip.y += math.sin(gantry_yaw)*maxForwardSearch
                        move_gantry(desired_probe_tip, gantry_yaw)

                    else:
                        print "we took too many probes! skip this config"
                        probe_limit_exceeded = True

            elif probe_plan_state == 3:

                print "\nPLAN STATE 3"
                print "-----------------------"

                if probe_sequence == probe_sequence_prev:

                    p = est.get_sparsest_point()

                    desired_probe_tip.x = p[0] - math.cos(gantry_yaw)*landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.y = p[1] - math.sin(gantry_yaw)*landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.z = -scorpion_gantry_offset_loc[2] + landmine_pos[2]
                    gantry_yaw = p[3]

                    move_gantry(desired_probe_tip, gantry_yaw)

                elif probe_sequence > probe_sequence_prev:

                    if (probe_sequence - probe_sequence_prev) < max_num_probes:
                        desired_probe_tip.x += math.cos(gantry_yaw)*maxForwardSearch
                        desired_probe_tip.y += math.sin(gantry_yaw)*maxForwardSearch
                        move_gantry(desired_probe_tip, gantry_yaw)

                    else:
                        print "we took too many probes! skip this config"
                        probe_limit_exceeded = True

            '''
            Execute Probing Procedure
            '''
            if not probe_limit_exceeded:

                probe_sequence += 1
                print "\nPROBE #", probe_sequence
                raw_input("\nPress Enter to continue...\n")

                probe_cmd_pub.publish(2) # start probing
                rospy.sleep(0.5) # give time for handshake
                while not probe_current_state.probe_complete: # while not finished
                    pass

            '''
            Advance States
            '''
            if probe_plan_state < 3:

                if est.point_count() > prev_point_count or probe_limit_exceeded:
                    probe_plan_state += 1 # advance
                    probe_sequence_prev = probe_sequence
                    prev_point_count = est.point_count()
                    probe_limit_exceeded = False

            if probe_plan_state == 3: # just exit here for now

                print est.print_results()

                raw_input("\nMove Probe Tip for Marking...\n")

                # TODO move probe tip to mine location for marking

                target = null_target
                probe_plan_state = -1
                probe_sequence = 0 # reset

                jetson_desired_state.publish(0) # go back to idle state

            # elif probe_plan_state == 3:
            #
            #     if not est.point_count() == prev_point_count or probe_limit_exceeded:
            #         probe_sequence_prev = probe_sequence
            #         prev_point_count = est.point_count()
            #         probe_limit_exceeded = False
            #
            #     if (est.point_count() >= num_contact_points
            #         and est.get_error() <= min_fit_error):
            #
            #         print est.print_results()
            #         target = null_target
            #         probe_plan_state = -1
            #         probe_sequence = 0 # reset

        r.sleep()

if __name__ == "__main__":
    main()

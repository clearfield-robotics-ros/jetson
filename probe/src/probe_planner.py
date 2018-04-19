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
from constraints_redo import Probe_Motion_Planner
from shapely.geometry import Point as shapely_Point

### monitor current state ###
current_state = 0 # if we don't get msgs
def update_state(data):
    global current_state
    current_state = data.data
jetson_current_state = rospy.Subscriber('current_state', Int16, update_state)

# def within_gantry_limits(loc, yaw):
#     pose_possible = False # start by assuming it's out of bounds 
#     candidate_trans = probe_to_gantry_transform(loc, yaw) # get transform for specified loc & yaw
#     if candidate_trans[0] >= 0 and candidate_trans[0] <= gantry_x_range: # if within both x bounds
#         if candidate_trans[1] >= 0 and candidate_trans[1] <= gantry_y_range: # and y bounds
#             pose_possible = True # switch the flag
#     return pose_possible

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
    print trans
    return trans

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
    possible_probe_approach_angles = np.linspace(gantry_th_min, gantry_th_max, 19)

    probe_motion_planner = Probe_Motion_Planner([0,0,0], [0,0,0]) # just instantiate with dummy points

    collision_free_points = []
    for angle in possible_probe_approach_angles:
        trans = probe_to_gantry_transform(desired_probe_tip, angle)
        point_to_check = shapely_Point(trans[1]/1000.0, angle) # Point takes (y[mm], th[rad])
        collision_free_points.append(probe_motion_planner.point_collision_free(point_to_check))
    print collision_free_points
    raw_input()


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

def move_sensor_head(pos, yaw):
    print "GANTRY YAW:", yaw*180/math.pi
    print "GANTRY POSIITON:", pos
    # plan, valid_plan = generate_probe_plan(pos, yaw) # only receives goal
    plan = generate_probe_plan(pos, yaw) # only receives goal
    # if valid_plan:
    for i in range(len(plan)): # go through each step of plan
        gantry_desired_state = to_gantry_msg()
        gantry_desired_state.state_desired       = 3
        gantry_desired_state.x_desired           = pos[0]
        gantry_desired_state.y_desired           = plan[i][0] * 1000.0
        gantry_desired_state.yaw_desired         = plan[i][1]
        # gantry_desired_state.probe_angle_desired = 0
        global gantry_desired_state_pub
        gantry_desired_state_pub.publish(gantry_desired_state)

        rospy.sleep(0.5) # give time for handshake

        while not gantry_current_status.position_reached: # block while not finished
            pass


def set_target(data):
    global target
    target = data
    print "target", target
    global probe_plan_state
    probe_plan_state = 0

def get_next_config(index):
    positions =  [[100, 400, 0],
                  [100, 780, 0.84],
                  [100, 700, -1.13]]
    return positions[index]

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


# def update_gantry_yaw(desired_yaw, probe_sequence, approach_angle_count): #, index_diff):
#     # returns the desired yaw and probe sequence directly
#     # also increments the counter for the unique approach angle
#     #   which is used for debugging the max number of probes in a row
#     approach_angle_count += 1
#     return desired_yaw, approach_angle_count, probe_sequence


def update_probe_contact(data):
    pass
    # if data.contact_made:
    #     (trans,rot) = listener.lookupTransform('/gantry', '/probe_tip', rospy.Time(0))
    #     global est
    #     est.add_point(trans[0], trans[1], trans[2])

def main():
    rospy.init_node('probe_planner')

    global probe_plan_state
    probe_plan_state = -1 # initial state

    # Transforms
    global listener
    listener = tf.TransformListener()


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
    rospy.Subscriber("/probe_teensy/probe_contact_reply", probe_data, update_probe_contact)
    global contact_block_flag
    contact_block_flag = False

    # Recieving Target from Metal Detector
    rospy.Subscriber("/set_probe_target", Point, set_target)
    null_target = Point()
    global target
    target = null_target

    global est
    est = Mine_Estimator(landmine_diameter, landmine_height)

    constraint_planning_test_index = 0
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
    test_motion = True

    r = rospy.Rate(100) # Hz
    while not rospy.is_shutdown():

        # check we're in the right state and have a Target
        if current_state == 4 and not target == null_target and test_motion:
            next_config = get_next_config(constraint_planning_test_index)
            move_sensor_head([next_config[0], next_config[1]], next_config[2])
            raw_input("\nPress Enter to move to next...\n")
            constraint_planning_test_index += 1
            # if constraint_planning_test_index == 3:
            #     test_motion = False

            # # '''
            # # Perform Planning for Probe
            # # '''

            #     print "\nPLAN STATE 0"
            #     print "-----------------------"

            #     if probe_sequence == 0:

            #         gantry_yaw = 0

            #         desired_probe_tip.x = target.x - landmine_diameter/2*probe_safety_factor
            #         desired_probe_tip.y = target.y
            #         desired_probe_tip.z = -scorpion_gantry_offset_loc[2] + landmine_pos[2]

            # #     print "PLAN STATE 0"
            # #     print "-----------------------"
            # #     if probe_sequence == 0:
            # #         # define desired probe tip position in gantry frame
            # #         desired_probe_tip.x = target.x - landmine_diameter/2*probe_safety_factor
            # #         desired_probe_tip.y = target.y
            # #         desired_probe_tip.z = -scorpion_gantry_offset_loc[2] + landmine_pos[2]
            # #         # gantry_yaw, approach_angle_count, parent_probe_seq = update_gantry_yaw(0, probe_sequence, approach_angle_count) # 0

            #     elif probe_sequence > 0:

            #         if probe_sequence < max_num_probes:
            #             desired_probe_tip.x += maxForwardSearch
            #             move_gantry(desired_probe_tip, gantry_yaw)

            #         else:
            #             print "we took too many probes! skip this config"
            #             probe_limit_exceeded = True

            # #         move_gantry(desired_probe_tip, gantry_yaw)

            #     print "\nPLAN STATE 1"
            #     print "-----------------------"

            #     if probe_sequence == probe_sequence_prev:

            #         gantry_yaw = +0.785398

            #         desired_probe_tip.x = target.x - math.cos(gantry_yaw)*landmine_diameter/2*probe_safety_factor
            #         desired_probe_tip.y = target.y - math.sin(gantry_yaw)*landmine_diameter/2*probe_safety_factor
            #         desired_probe_tip.z = -scorpion_gantry_offset_loc[2] + landmine_pos[2]

            # #         desired_probe_tip.x = plan_x - math.cos(gantry_yaw)*landmine_diameter/2*probe_safety_factor
            # #         desired_probe_tip.y = plan_y - math.sin(gantry_yaw)*landmine_diameter/2*probe_safety_factor#-200
            # #         desired_probe_tip.z = -scorpion_gantry_offset_loc[2] + landmine_pos[2]

            # #         move_gantry(desired_probe_tip, gantry_yaw)


            #         if (probe_sequence - probe_sequence_prev) < max_num_probes:
            #             desired_probe_tip.x += math.cos(gantry_yaw)*maxForwardSearch
            #             desired_probe_tip.y += math.sin(gantry_yaw)*maxForwardSearch
            #             move_gantry(desired_probe_tip, gantry_yaw)

            #         else:
            #             print "we took too many probes! skip this config"
            #             probe_limit_exceeded = True

            # # elif probe_plan_state == 2:


            #     print "\nPLAN STATE 2"
            #     print "-----------------------"

            #     if probe_sequence == probe_sequence_prev:

            #         gantry_yaw = -0.785398

            #         desired_probe_tip.x = target.x - math.cos(gantry_yaw)*landmine_diameter/2*probe_safety_factor
            #         desired_probe_tip.y = target.y - math.sin(gantry_yaw)*landmine_diameter/2*probe_safety_factor
            #         desired_probe_tip.z = -scorpion_gantry_offset_loc[2] + landmine_pos[2]

            #         move_gantry(desired_probe_tip, gantry_yaw)

            #     elif probe_sequence > probe_sequence_prev:

            #         if (probe_sequence - probe_sequence_prev) < max_num_probes:
            #             desired_probe_tip.x += math.cos(gantry_yaw)*maxForwardSearch
            #             desired_probe_tip.y += math.sin(gantry_yaw)*maxForwardSearch
            #             move_gantry(desired_probe_tip, gantry_yaw)

            #         else:
            #             print "we took too many probes! skip this config"
            #             probe_limit_exceeded = True

            # elif probe_plan_state == 3:

            #     print "\nPLAN STATE 3"
            #     print "-----------------------"

            # #     if probe_sequence == probe_sequence_prev: # if this is the first for this approach angle ??

            # #         p = est.get_sparsest_point()
            # #         # print "cos yaw: ", math.cos(gantry_yaw)
            # #         desired_probe_tip.x = p[0] - math.cos(gantry_yaw)*landmine_diameter/2*probe_safety_factor
            # #         desired_probe_tip.y = p[1] - math.sin(gantry_yaw)*landmine_diameter/2*probe_safety_factor
            # #         desired_probe_tip.z = -scorpion_gantry_offset_loc[2] + landmine_pos[2]
            # #         # print "Desired probe tip: ", desired_probe_tip
            # #         gantry_yaw, approach_angle_count, parent_probe_seq = update_gantry_yaw(p[3], probe_sequence, approach_angle_count) # p[3]

            # #         move_gantry(desired_probe_tip, gantry_yaw)

            # #     elif probe_sequence > probe_sequence_prev:# and not one_approach_angle_finished:  # if you want to keep advancing along the same apch angle

            # #         desired_probe_tip.x += math.cos(gantry_yaw)*maxForwardSearch
            # #         desired_probe_tip.y += math.sin(gantry_yaw)*maxForwardSearch


            #     if (probe_sequence - probe_sequence_prev) < max_num_probes:
            #         desired_probe_tip.x += math.cos(gantry_yaw)*maxForwardSearch
            #         desired_probe_tip.y += math.sin(gantry_yaw)*maxForwardSearch
            #         move_gantry(desired_probe_tip, gantry_yaw)

            #     else:
            #         print "we took too many probes! skip this config"
            #         probe_limit_exceeded = True

            '''
            Execute Probing Procedure
            '''
            # if not probe_limit_exceeded:

            #     probe_sequence += 1
            #     print "\nPROBE #", probe_sequence
            #     raw_input("\nPress Enter to continue...\n")

            #     probe_cmd_pub.publish(2) # start probing
            #     rospy.sleep(0.5) # give time for handshake
            #     while not probe_current_state.probe_complete: # while not finished
            #         pass

            # '''
            # Advance States
            # '''
            # if probe_plan_state < 3:

            #     if est.point_count() > prev_point_count or probe_limit_exceeded:
            #         probe_plan_state += 1 # advance
            #         probe_sequence_prev = probe_sequence
            #         prev_point_count = est.point_count()
            #         probe_limit_exceeded = False

            # if probe_plan_state == 3:
            #     while True:
            #         pass

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
            #
            #         ### TODO: Change State in Jetson
            #
            #         target = null_target
            #         probe_plan_state = -1
            #         probe_sequence = 0 # reset

        r.sleep()

if __name__ == "__main__":
    main()

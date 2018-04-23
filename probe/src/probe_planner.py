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

### monitor current state ###
current_state = 0 # if we don't get msgs
def update_state(data):
    global current_state
    current_state = data.data
jetson_current_state = rospy.Subscriber('current_state', Int16, update_state)

global gantry_yaw_history
gantry_yaw_history = [0, 0]

def probe_to_gantry_transform(loc,yaw,state):
    global br

    if state == 'probe':

        Hprobe = np.array([[1,0,0,loc.x],
                           [0,1,0,loc.y],
                           [0,0,1,loc.z],
                           [0,0,0,1]])

        '''
        test = Hprobe
        br.sendTransform((test[0][3],test[1][3],test[2][3]),
           tf.transformations.quaternion_from_euler(0,0,0),
           rospy.Time.now(),
           "target_probe",
           "gantry")
        '''

        Hyaw = np.array([[math.cos(yaw),-math.sin(yaw),0,0],
                         [math.sin(yaw),math.cos(yaw),0,0],
                         [0,0,1,0],
                         [0,0,0,1]])

        Hyrot = np.array([[math.cos(-probe_angle),0,math.cos(-probe_angle),0],
                          [0,1,0,0],
                          [math.sin(-probe_angle),0,math.cos(-probe_angle),0],
                          [0,0,0,1]])

        Hd = np.array([[1,0,0,-(probe_length+max_probe_distance)],
                       [0,1,0,0],
                       [0,0,1,0],
                       [0,0,0,1]])

        '''
        test = Hprobe.dot(Hyaw).dot(Hyrot).dot(Hd)
        br.sendTransform((test[0][3],test[1][3],test[2][3]),
          tf.transformations.quaternion_from_euler(0,0,0),
          rospy.Time.now(),
          "target_probe_base",
          "gantry")
        '''

        Hoffset = np.array([[1,0,0,-sensorhead_probebase_offset_loc[0]],
                           [0,1,0,-sensorhead_probebase_offset_loc[1]],
                           [0,0,1,-sensorhead_probebase_offset_loc[2]],
                           [0,0,0,1]])

        H = Hprobe.dot(Hyaw).dot(Hoffset).dot(Hyrot).dot(Hd)

        '''
        test = H
        br.sendTransform((test[0][3],test[1][3],test[2][3]),
          tf.transformations.quaternion_from_euler(0,0,0),
          rospy.Time.now(),
          "target_sensor_head",
          "gantry")
        '''

    elif state == 'mark':

        Hprobe = np.array([[1,0,0,loc.x],
                           [0,1,0,loc.y],
                           [0,0,1,loc.z],
                           [0,0,0,1]])

        '''
        test = Hprobe
        br.sendTransform((test[0][3],test[1][3],test[2][3]),
           tf.transformations.quaternion_from_euler(0,0,0),
           rospy.Time.now(),
           "target_probe",
           "gantry")
        '''

        Hyaw = np.array([[math.cos(yaw),-math.sin(yaw),0,0],
                         [math.sin(yaw),math.cos(yaw),0,0],
                         [0,0,1,0],
                         [0,0,0,1]])


        Hd = np.array([[1,0,0,-sensorhead_marker_offset_loc[0] + sensorhead_probebase_offset_loc[0]], \
                        [0,1,0,-sensorhead_marker_offset_loc[1] + sensorhead_probebase_offset_loc[1]], \
                        [0,0,1,-sensorhead_marker_offset_loc[2] + sensorhead_probebase_offset_loc[2]], \
                        [0,0,0,1]])

        '''
        test = Hprobe.dot(Hyaw).dot(Hd)
        br.sendTransform((test[0][3],test[1][3],test[2][3]),
          tf.transformations.quaternion_from_euler(0,0,0),
          rospy.Time.now(),
          "target_probe_base",
          "gantry")
        '''

        Hoffset = np.array([[1,0,0,-sensorhead_probebase_offset_loc[0]],
                           [0,1,0,-sensorhead_probebase_offset_loc[1]],
                           [0,0,1,-sensorhead_probebase_offset_loc[2]],
                           [0,0,0,1]])

        H = Hprobe.dot(Hyaw).dot(Hoffset).dot(Hd)

        '''
        test = H
        br.sendTransform((test[0][3],test[1][3],test[2][3]),
          tf.transformations.quaternion_from_euler(0,0,0),
          rospy.Time.now(),
          "target_sensor_head",
          "gantry")
          '''

    trans = np.matmul(H,np.array([[0],[0],[0],[1]]))
    return trans

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

        gantry_yaw_history.append(gantry_desired_state.yaw_desired)
        rotation_rate = 1.0 # rad/s (about 90deg/sec)
        gantry_yaw_delay_factor = 1.0 # use this for tuning
        # print "gantry_yaw_history", gantry_yaw_history
        yaw_delta = gantry_yaw_history[-1] - gantry_yaw_history[-2] # difference of last and second last
        yaw_timeout = abs(yaw_delta * rotation_rate * gantry_yaw_delay_factor) # seconds
        yaw_delay_timer_start = time.time()
        print "yaw_timeout", yaw_timeout
        
        while not gantry_current_status.position_reached or (time.time()-yaw_delay_timer_start)<yaw_timeout: # block while not finished
            pass

def move_gantry(desired_probe_tip, gantry_yaw, state):

    print "yaw:", gantry_yaw*180/math.pi
    print desired_probe_tip

    # work out gantry carriage position
    trans = probe_to_gantry_transform(desired_probe_tip, gantry_yaw, state)

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

        gantry_yaw_history.append(gantry_desired_state.yaw_desired)
        rotation_rate = 1.5 # rad/s (about 90deg/sec)
        gantry_yaw_delay_factor = 1 # use this for tuning
        # print "gantry_yaw_history", gantry_yaw_history
        yaw_delta = gantry_yaw_history[-1] - gantry_yaw_history[-2] # difference of last and second last
        yaw_timeout = abs(yaw_delta * rotation_rate * gantry_yaw_delay_factor) # seconds
        yaw_delay_timer_start = time.time()
        print "yaw_timeout", yaw_timeout

        while not gantry_current_status.position_reached or (time.time()-yaw_delay_timer_start)<yaw_timeout: # block while not finished
            pass

def generate_probe_plan(goal_trans, goal_rot):
    (current_trans, current_rot) = listener.lookupTransform('gantry', 'sensor_head', rospy.Time(0))
    current_rot_euler = tf.transformations.euler_from_quaternion(current_rot)[2]
    start_config = [current_trans[0], current_trans[1], current_rot_euler]
    end_config = [goal_trans[0], goal_trans[1], goal_rot]
    print "start config", start_config
    print "end config", end_config
    probe_motion_planner = Probe_Motion_Planner(start_config, end_config, gantry_limits)
    path = probe_motion_planner.plan_path()
    plan_arrays = [[step.x, step.y] for step in path] # reconfigure into an array of arrays
    print plan_arrays
    return plan_arrays

def get_next_config(index):
    positions =  [[100, 400, 0],
                  [100, 600, -1.13],
                  # [100, 780, 0.84],
                  [100, 780, 1.0]]
    return positions[index]

def calc_probe_angle_range(desired_probe_tip, state):
    gantry_th_min                  = rospy.get_param('gantry_th_min')
    gantry_th_max                  = rospy.get_param('gantry_th_max')
    possible_probe_approach_angles = np.linspace(gantry_th_min, gantry_th_max, 90) # 10deg increments for 180deg 

    x_mid = (gantry_limits[0] + gantry_limits[1])/2 # so that it doesn't run into boundary issues
    y_mid = (gantry_limits[2] + gantry_limits[3])/2

    probe_motion_planner = Probe_Motion_Planner([x_mid,y_mid,0], [x_mid,y_mid,0], gantry_limits) # just instantiate with dummy points to be able to access functions

    collision_free_points = []
    for angle in possible_probe_approach_angles:
        trans = probe_to_gantry_transform(desired_probe_tip, angle, state)
        # print "trans", trans
        point_to_check = shapely_Point(trans[1]/1000.0, angle) # Point takes (y[mm], th[rad])
        collision_free_points.append(probe_motion_planner.point_collision_free(point_to_check))
    # tog = zip(possible_probe_approach_angles, collision_free_points)
    # print "tog", tog

    ### MAX CONTIGUOUS HACK
    true_indices = [i for i, x in enumerate(collision_free_points) if x==1]
    # print true_indices

    index_diff = [true_indices[i+1]-true_indices[i] for i in range(len(true_indices)-1)]
    # print index_diff

    # indices where not 1
    gap_indices = [i+1 for i, x in enumerate(index_diff) if x != 1]
    # print gap_indices

    # add start and end indices
    gap_indices.insert(0,0)
    gap_indices.append(len(true_indices))
    # print gap_indices

    max_range = [gap_indices[i+1]-gap_indices[i] for i in range(len(gap_indices)-1)]
    # print max_range

    # get the max range
    richest_zone_index = np.argmax(max_range)
    # print richest_zone_index

    richest_zone = true_indices[gap_indices[richest_zone_index]:gap_indices[richest_zone_index+1]]
    # print richest_zone

    mask = []
    # do a mask with just the ones in question
    for idx in range(len(possible_probe_approach_angles)):
        if idx in richest_zone:
            mask.append(1)
        else:
            mask.append(0)

    # print mask

    tog = zip(possible_probe_approach_angles, mask)
    # print tog
    ### 

    allowable_angles = [x[0] for x in tog if x[1]==True] # only extract collision-free points
    print "allowable_angles", allowable_angles
    # assumption! contiguous collision free points i.e. if the first is at 20deg, last at 160deg, then all of 20-160deg is free
    if len(allowable_angles) == 0:
        return []
    else:
        return [allowable_angles[0], allowable_angles[-1]]

def generate_probe_angle_sequence(target, state):
# def generate_probe_angle_sequence(index):
    num_unique_probe_angles = 3
    # angle_range = calc_probe_angle_range(get_desired_probe_tip(index)) # for testing with dummy gantry positions
    angle_range = calc_probe_angle_range(target, state)
    proportions = [0.5, 0.0, 1.0] # proportions between the lower and upper limits
    angle_sequence = [p*(angle_range[1]-angle_range[0])+angle_range[0] for p in proportions]
    return angle_sequence, num_unique_probe_angles


def set_target(data):
    global target
    target = data
    print "Metal Detector Target:\n", target
    global probe_plan_state
    probe_plan_state = 0


def update_gantry_state(data):
    global gantry_current_status
    gantry_current_status = data
    global gantry_limits
    gantry_limits = [data.x_min, data.x_max, data.y_min, data.y_max]


def update_probe_state(data):
    global probe_current_state
    probe_current_state = data

    global contact_block_flag
    if not data.probe_complete and data.contact_made and not contact_block_flag:

        print "\nNEW CONTACT POINT!\n"

        (trans,rot) = listener.lookupTransform('/gantry', '/probe_tip', rospy.Time(0))
        global est
        est_mine_list[-1].add_point(trans[0], trans[1], trans[2])
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
    global br
    br = tf.TransformBroadcaster()

    # Parameters
    landmine_burial_depth           = rospy.get_param('landmine_burial_depth')
    landmine_diameter               = rospy.get_param('landmine_diameter')
    landmine_height                 = rospy.get_param('landmine_height')
    global sensorhead_probebase_offset_loc
    sensorhead_probebase_offset_loc = rospy.get_param('sensorhead_probebase_offset_loc')
    probe_safety_factor             = rospy.get_param('probe_safety_factor')
    global probe_angle
    probe_angle                     = rospy.get_param('sensorhead_probebase_offset_rot')[1]
    scorpion_gantry_offset_loc      = rospy.get_param('scorpion_gantry_offset_loc')

    global probe_length
    probe_length                    = rospy.get_param('probe_length')
    global max_probe_distance
    max_probe_distance              = rospy.get_param('max_probe_distance')
    max_probe_depth                 = sensorhead_probebase_offset_loc[2] - math.sin(probe_angle) * \
                                        (max_probe_distance + probe_length)

    maxForwardSearch                = math.cos(probe_angle)*landmine_height
    global sensorhead_marker_offset_loc
    sensorhead_marker_offset_loc    = rospy.get_param('sensorhead_marker_offset_loc')
    gantry_width                    = rospy.get_param('gantry_width')
    num_contact_points              = rospy.get_param('num_contact_points')
    min_fit_error                   = rospy.get_param('min_fit_error')
    max_num_probes                  = rospy.get_param('max_num_probes')

    probe_limit_exceeded            = False
    prev_point_count                = 0

    # Jetson Messages
    jetson_desired_state = rospy.Publisher('/desired_state', Int16, queue_size=10)
    jetson_desired_mine = rospy.Publisher('/desired_mine', Int16, queue_size=10)

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

    # mine estimator object
    global est_mine_list
    est_mine_list = []
    est_mine_list.append(Mine_Estimator(landmine_diameter, landmine_height))

    constraint_planning_test_index = 0
    test_motion = True
    # DEBUG
    # N = 4
    # rospy.sleep(0.5) # Sleeps for 1 sec
    # for i in range(0,N):
    #     x = -landmine_diameter/2*math.sin( (100 + 120/(N-1)*i) * math.pi/180)+250
    #     y = landmine_diameter/2*math.cos( (100 + 120/(N-1)*i) * math.pi/180)
    #
    #     est_mine_list[-1].add_point(x,y,0)
    #
    # rospy.sleep(0.5) # Sleeps for 1 sec
    # p = est_mine_list[-1].get_sparsest_point()
    #
    # pdb.set_trace()

    r = rospy.Rate(100) # Hz
    while not rospy.is_shutdown():

        # check we're in the right state and have a Target
        if current_state == 4 and not target == null_target:

        ### START COMMENTED OUT SECTION FOR CONSTRAINTS SIMULATION ###    
        # if current_state == 4 and not target == null_target and test_motion:
        #     next_config = get_next_config(constraint_planning_test_index)
        #     move_sensor_head([next_config[0], next_config[1]], next_config[2])
        #     raw_input("\nPress Enter to move to next...\n")
        #     constraint_planning_test_index += 1
        #     if constraint_planning_test_index == 3:
        #         test_motion = False
        ### END COMMENTED OUT SECTION FOR CONSTRAINTS SIMULATION ###    

        ### START COMMENTED OUT SECTION ###
            '''
            Perform Planning for Probe
            '''

            angle_sequence, num_unique_probe_angles = generate_probe_angle_sequence(target, 'probe')

            if probe_plan_state == 0:

                print "\nPLAN STATE 0"
                print "-----------------------"

                if probe_sequence == 0:

                    # gantry_yaw = 0
                    gantry_yaw = angle_sequence[0]


                    desired_probe_tip.x = target.x - landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.y = target.y
                    desired_probe_tip.z = max_probe_depth
                    move_gantry(desired_probe_tip, gantry_yaw, 'probe')

                elif probe_sequence > 0:

                    if probe_sequence < max_num_probes:
                        desired_probe_tip.x += maxForwardSearch
                        move_gantry(desired_probe_tip, gantry_yaw, 'probe')

                    else:
                        print "we took too many probes! skip this config"
                        probe_limit_exceeded = True

            elif probe_plan_state == 1:

                print "\nPLAN STATE 1"
                print "-----------------------"

                if probe_sequence == probe_sequence_prev:

                    # gantry_yaw = +0.785398
                    gantry_yaw = angle_sequence[1]

                    desired_probe_tip.x = target.x - math.cos(gantry_yaw)*landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.y = target.y - math.sin(gantry_yaw)*landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.z = max_probe_depth
                    move_gantry(desired_probe_tip, gantry_yaw, 'probe')

                elif probe_sequence > probe_sequence_prev:

                    if (probe_sequence - probe_sequence_prev) < max_num_probes:
                        desired_probe_tip.x += math.cos(gantry_yaw)*maxForwardSearch
                        desired_probe_tip.y += math.sin(gantry_yaw)*maxForwardSearch
                        move_gantry(desired_probe_tip, gantry_yaw, 'probe')

                    else:
                        print "we took too many probes! skip this config"
                        probe_limit_exceeded = True

            elif probe_plan_state == 2:

                print "\nPLAN STATE 2"
                print "-----------------------"

                if probe_sequence == probe_sequence_prev:

                    # gantry_yaw = -0.785398
                    gantry_yaw = angle_sequence[2]

                    desired_probe_tip.x = target.x - math.cos(gantry_yaw)*landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.y = target.y - math.sin(gantry_yaw)*landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.z = max_probe_depth
                    move_gantry(desired_probe_tip, gantry_yaw, 'probe')

                elif probe_sequence > probe_sequence_prev:

                    if (probe_sequence - probe_sequence_prev) < max_num_probes:
                        desired_probe_tip.x += math.cos(gantry_yaw)*maxForwardSearch
                        desired_probe_tip.y += math.sin(gantry_yaw)*maxForwardSearch
                        move_gantry(desired_probe_tip, gantry_yaw, 'probe')

                    else:
                        print "we took too many probes! skip this config"
                        probe_limit_exceeded = True

            elif probe_plan_state == 3: # CAUTION! ANGLE SEQUENCE CURRENTLY ONLY HAS 3 ELEMENTS!

                print "\nPLAN STATE 3"
                print "-----------------------"

                if probe_sequence == probe_sequence_prev:

                    p = est_mine_list[-1].get_sparsest_point()
                    desired_probe_tip.x = p[0] - math.cos(gantry_yaw)*landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.y = p[1] - math.sin(gantry_yaw)*landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.z = max_probe_depth
                    gantry_yaw = p[3]
                    move_gantry(desired_probe_tip, gantry_yaw, 'probe')

                elif probe_sequence > probe_sequence_prev:

                    if (probe_sequence - probe_sequence_prev) < max_num_probes:
                        desired_probe_tip.x += math.cos(gantry_yaw)*maxForwardSearch
                        desired_probe_tip.y += math.sin(gantry_yaw)*maxForwardSearch
                        move_gantry(desired_probe_tip, gantry_yaw, 'probe')

                    else:
                        print "we took too many probes! skip this config"
                        probe_limit_exceeded = True

            '''
            Execute Probing Procedure
            '''
            if not probe_limit_exceeded:

                probe_sequence += 1
                print "\nPROBE #", probe_sequence
                raw_input("\nPress Enter to probe...\n")

                probe_cmd_pub.publish(2) # start probing
                rospy.sleep(0.5) # give time for handshake
                while not probe_current_state.probe_complete: # while not finished
                    pass

            '''
            Advance States
            '''
            if probe_plan_state < 3:

                if est_mine_list[-1].point_count() > prev_point_count or probe_limit_exceeded:
                    probe_plan_state += 1 # advance
                    probe_sequence_prev = probe_sequence
                    prev_point_count = est_mine_list[-1].point_count()
                    probe_limit_exceeded = False

            if probe_plan_state == 3: # just exit here for now

                est_mine_list[-1].print_results()

                raw_input("\nMove Probe Tip for Marking...\n")

                gantry_yaw = angle_sequence[0]
                desired_probe_tip.x = est_mine_list[-1].c_x
                desired_probe_tip.y = est_mine_list[-1].c_y
                desired_probe_tip.z = sensorhead_marker_offset_loc[2] #max_probe_depth
                move_gantry(desired_probe_tip, gantry_yaw, 'mark')

                # Reset everyhting before we go go to the next mine
                target = null_target
                probe_plan_state = -1
                probe_sequence = 0
                prev_point_count = 0
                jetson_desired_state.publish(0) # go back to idle state
                jetson_desired_mine.publish(0) # increment mine count
                est_mine_list.append(Mine_Estimator(landmine_diameter, landmine_height))

            # elif probe_plan_state == 3:
            #
            #     if not est_mine_list[-1].point_count() == prev_point_count or probe_limit_exceeded:
            #         probe_sequence_prev = probe_sequence
            #         prev_point_count = est_mine_list[-1].point_count()
            #         probe_limit_exceeded = False
            #
            #     if (est_mine_list[-1].point_count() >= num_contact_points
            #         and est_mine_list[-1].get_error() <= min_fit_error):
            #
            #         est_mine_list[-1].print_results()
                    # target = null_target
                    # probe_plan_state = -1
                    # probe_sequence = 0 # reset
                    # jetson_desired_state.publish(0) # go back to idle state
                    # jetson_desired_mine.publish(0) # increment mine count
                    # est_mine_list.append(Mine_Estimator(landmine_diameter, landmine_height)) # looking for new mine now!

        ### END COMMENTED OUT SECTION #

        r.sleep()

if __name__ == "__main__":
    main()

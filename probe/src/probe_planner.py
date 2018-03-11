#!/usr/bin/env python

import rospy
import numpy as np
import tf
import math
import pdb
from geometry_msgs.msg import Point
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
from visualization_msgs.msg import Marker
from mine_estimator import Mine_Estimator

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

    Hoffset = np.array([[1,0,0,-probe_base_offset_loc[0]],
                       [0,1,0,-probe_base_offset_loc[1]],
                       [0,0,1,-probe_base_offset_loc[2]],
                       [0,0,0,1]])

    H = Hprobe.dot(Hyaw).dot(Hoffset).dot(Hyrot).dot(Hd)
    trans = np.matmul(H,np.array([[0],[0],[0],[1]]))

    return trans

def move_gantry(desired_probe_tip, gantry_yaw):

    print "MOVE GANTRY TO:", desired_probe_tip
    plot_point(desired_probe_tip.x,desired_probe_tip.y,desired_probe_tip.z,[0,1,0],30)

    # work out gantry carriage position
    trans = probe_to_gantry_transform(desired_probe_tip, gantry_yaw)

    # send gantry position message
    gantry_desired_state_msg = Int16MultiArray()
    gantry_desired_state_msg.data = [
        3,                               # 0: desired state = pos control
        0,                               # 1: sweep velocity
        trans[0],                        # 2: x Position
        trans[1],                        # 3: y position
        gantry_yaw*180/math.pi,          # 4: yaw angle (deg)
        0]                               # 5: probe yaw angle
    global gantry_desired_state_pub
    gantry_desired_state_pub.publish(gantry_desired_state_msg)

    rospy.sleep(0.5) # give time for handshake

    while not gantry_current_state[6] == 1: # while not finished
        pass

def set_target(data):
    global target
    target = data
    print "target", target

def update_gantry_state(data):
    global gantry_current_state
    gantry_current_state = data.data

def update_probe_state(data):
    global probe_current_state
    probe_current_state = data.data

contact_viz_id = 0
def plot_point(x,y,z,col,size):
    # Visualize probe point
    global contact_viz_id
    global contact_viz_pub
    msg = Marker()
    msg.header.frame_id = "gantry"
    msg.header.seq = contact_viz_id
    msg.header.stamp = rospy.Time.now()
    msg.ns = "probe_plan_viz"
    msg.id = contact_viz_id
    msg.type = 2  # cube
    msg.action = 0  # add
    msg.pose.position = Point(x,y,z)
    msg.pose.orientation.w = 1
    msg.scale.x = size
    msg.scale.y = size
    msg.scale.z = size
    msg.color.a = 0.25
    msg.color.r = col[0]
    msg.color.g = col[1]
    msg.color.b = col[2]
    contact_viz_pub.publish(msg)
    contact_viz_id += 1


def update_probe_contact(data):
    (trans,rot) = listener.lookupTransform('/gantry', '/probe_tip', rospy.Time(0))
    global est
    est.add_point(trans[0], trans[1], trans[2])


def main():
    rospy.init_node('probe_planner')
    probe_plan_state = 0 # initial state

    # Transforms
    global br
    br = tf.TransformBroadcaster()
    global listener
    listener = tf.TransformListener()

    # Parameters
    landmine_pos = rospy.get_param('landmine_pos')
    landmine_diameter = rospy.get_param('landmine_diameter')
    landmine_height = rospy.get_param('landmine_height')
    global probe_base_offset_loc
    probe_base_offset_loc = rospy.get_param('probe_base_offset_loc')
    probe_safety_factor = rospy.get_param('probe_safety_factor')
    global probe_angle
    probe_angle = rospy.get_param('probe_base_offset_rot')[1]
    scorpion_gantry_offset_loc = rospy.get_param('scorpion_gantry_offset_loc')
    global probe_length
    probe_length = (scorpion_gantry_offset_loc[2] + probe_base_offset_loc[2]
        + abs(landmine_pos[2])) / math.sin(probe_angle)
    maxForwardSearch = math.cos(probe_angle)*landmine_height*(1/probe_safety_factor);
    gantry_width = rospy.get_param('gantry_width')

    # blocking variables
    global set_desired_gantry_pose
    set_desired_gantry_pose = False
    set_probe = False
    probe_sequence = 0

    # Gantry Control Messages
    global gantry_desired_state_pub
    gantry_desired_state_pub = rospy.Publisher("/gantry_desired_state",Int16MultiArray,queue_size=10)
    sub2 = rospy.Subscriber("/gantry_current_state", Int16MultiArray, update_gantry_state)

    # Probe Related Messages
    probe_cmd_pub = rospy.Publisher("/probe_cmd_send", Int16, queue_size=10)
    desired_probe_tip = Point()
    gantry_yaw = 0
    sub3 = rospy.Subscriber("/probe_status_reply", Int16MultiArray, update_probe_state)
    sub4 = rospy.Subscriber("/probe_contact_reply", Int16MultiArray, update_probe_contact)

    global contact_viz_pub
    contact_viz_pub = rospy.Publisher('probe_contact_viz', Marker, queue_size=10)

    # Recieving Target from Metal Detector
    sub = rospy.Subscriber("/set_probe_target", Point, set_target)
    null_target = Point()
    global target
    target = null_target

    global est
    est = Mine_Estimator(landmine_diameter, landmine_height)

    # DEBUG
    # Gantry: 1200,-400,300
    # landmine_pos: [1800, 100, -150]

    # target.x = 600
    # target.y = 500
    # target.z = -450

    # DEBUG
    # rospy.sleep(1) # Sleeps for 1 sec
    # for i in range(0,10):
    #     x = -landmine_diameter/2*math.sin(180/10*i * math.pi/180)+250
    #     y = landmine_diameter/2*math.cos(180/10*i * math.pi/180)
    #
    #     est.add_point(x,y,0)
    #
    #     rospy.sleep(0.5)
    #
    # print(est.get_est())
    # pdb.set_trace()
    # DEBUG


    r = rospy.Rate(100) # Hz
    while not rospy.is_shutdown():

        # check we're in the right state and have a Target
        if current_state == 4 and not target == null_target:

            '''
            Perform Planning for Probe
            '''
            if probe_plan_state == 0:

                print "PLAN STATE 0"

                if probe_sequence == 0:
                    # define desired probe tip position in gantry frame
                    desired_probe_tip.x = target.x - landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.y = target.y
                    desired_probe_tip.z = -scorpion_gantry_offset_loc[2] + landmine_pos[2] # set to depth
                    gantry_yaw = 0

                    move_gantry(desired_probe_tip, gantry_yaw)
                    probe_sequence += 1

                elif probe_sequence > 0:
                    desired_probe_tip.x += maxForwardSearch

                    move_gantry(desired_probe_tip, gantry_yaw)
                    probe_sequence += 1

            elif probe_plan_state == 1:

                print "PLAN STATE 1"

                if probe_sequence == 0:

                    # define desired probe tip position in gantry frame
                    th = np.array([-math.pi/4,0.,math.pi/4])
                    x = landmine_diameter/2*np.cos(th) + est.most_recent_point().x
                    y = landmine_diameter/2*np.sin(th) + est.most_recent_point().y
                    z = np.ones(len(th))*est.most_recent_point().z

                    # DEBUG
                    # for i in range(0,len(th)):
                    #     plot_point(x[i],y[i],z[i],[1,0,0],10)
                    # DEBUG

                    if target.y > gantry_width/2:
                        gantry_yaw = th[2] # get desired yaw
                        plan_x = x[0]
                        plan_y = y[0]
                    else:
                        gantry_yaw = th[0] # get desired yaw
                        plan_x = x[2]
                        plan_y = y[2]

                    desired_probe_tip.x = plan_x - math.cos(gantry_yaw)*landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.y = plan_y - math.sin(gantry_yaw)*landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.z = -scorpion_gantry_offset_loc[2] + landmine_pos[2] # set to depth

                    move_gantry(desired_probe_tip, gantry_yaw)
                    probe_sequence += 1

                elif probe_sequence > 0:

                    desired_probe_tip.x += math.cos(gantry_yaw)*maxForwardSearch
                    desired_probe_tip.y += math.sin(gantry_yaw)*maxForwardSearch

                    move_gantry(desired_probe_tip, gantry_yaw)
                    probe_sequence += 1

            elif probe_plan_state == 2:

                print "PLAN STATE 2"
                while True:
                    pass

            '''
            Execute Probing Procedure
            '''
            print "BEGIN PROBING"
            probe_cmd_pub.publish(2) # start probing
            rospy.sleep(0.5) # give time for handshake
            while not probe_current_state[2] == 1: # while not finished
                pass

            '''
            Advance States
            '''
            if probe_plan_state == 0:
                if est.point_count() > 0:
                    probe_plan_state = 1 # advance
                    probe_sequence = 0 # reset

            elif probe_plan_state == 1:
                if est.point_count() > 1:
                    probe_plan_state = 2 # advance
                    probe_sequence = 0 # reset

        r.sleep()

if __name__ == "__main__":
    main()

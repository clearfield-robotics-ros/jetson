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

    H = np.matmul(np.matmul(np.matmul(Hprobe,Hyaw),Hyrot),Hd)
    test = np.matmul(H,np.array([[0],[0],[0],[1]]))

    (trans,rot) = listener.lookupTransform('/probe_base', '/sensor_head', rospy.Time(0))

    Hoffset = np.array([[1,0,0,-probe_base_offset_loc[0]],
                       [0,1,0,-probe_base_offset_loc[1]],
                       [0,0,1,-probe_base_offset_loc[2]],
                       [0,0,0,1]])

    H = np.matmul(np.matmul(np.matmul(np.matmul(Hoffset,Hprobe),Hyaw),Hyrot),Hd)
    trans = np.matmul(H,np.array([[0],[0],[0],[1]]))

    return trans

def move_gantry(desired_probe_tip, gantry_yaw):
    # work out gantry carriage position
    trans = probe_to_gantry_transform(desired_probe_tip, gantry_yaw)

    # send gantry position message
    gantry_desired_state_msg = Int16MultiArray()
    gantry_desired_state_msg.data = [
        3,                               # 0: desired state = pos control
        0,                               # 1: sweep velocity
        trans[0],                        # 2: x Position
        trans[1],                        # 3: y position
        gantry_yaw,                      # 4: yaw angle
        0]                               # 5: probe yaw angle

    global gantry_desired_state_pub
    gantry_desired_state_pub.publish(gantry_desired_state_msg)

    global set_desired_gantry_pose
    set_desired_gantry_pose = True

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
def plot_point(x,y,z):
    # Visualize probe point
    global contact_viz_id
    global contact_viz_pub
    msg = Marker()
    # msg.header.frame_id = "gantry"
    msg.header.frame_id = "probe_tip" # only for debug
    msg.header.seq = contact_viz_id
    msg.header.stamp = rospy.Time.now()
    msg.ns = "probe_contact_viz"
    msg.id = contact_viz_id
    msg.type = 2  # cube
    msg.action = 0  # add
    msg.pose.position = Point(x,y,z)
    msg.pose.orientation.w = 1
    msg.scale.x = 10
    msg.scale.y = 10
    msg.scale.z = 10
    msg.color.a = 1.0
    msg.color.r = 1.0
    msg.color.g = 0.0
    msg.color.b = 0.0
    contact_viz_pub.publish(msg)
    contact_viz_id += 1


def update_probe_contact(data):
    (trans,rot) = listener.lookupTransform('/gantry', '/probe_tip', rospy.Time(0))
    est.add_point(trans[0], trans[1], trans[2])

'''
Probe Planner States
0 - Initial Search
1 - 45deg Search
2 - Max Info Search
'''
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
    probe_length = (scorpion_gantry_offset_loc[2] + probe_base_offset_loc[2] + abs(landmine_pos[2])) / math.sin(probe_angle)
    maxForwardSearch = math.cos(probe_angle)*landmine_height*(1/probe_safety_factor);
    gantry_width = rospy.get_param('gantry_width')

    # blocking variables
    global set_desired_gantry_pose
    set_desired_gantry_pose = False
    set_probe = False

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

    est = Mine_Estimator(landmine_diameter, landmine_height)

    # DEBUG
    rospy.sleep(1) # Sleeps for 1 sec
    for i in range(0,10):
        x = -landmine_diameter/2*math.sin(180/10*i * math.pi/180)+250
        y = landmine_diameter/2*math.cos(180/10*i * math.pi/180)

        est.add_point(x,y,0)

        rospy.sleep(0.5)

    print(est.get_est())

    pdb.set_trace()
    # DEBUG


    r = rospy.Rate(100) # Hz
    while not rospy.is_shutdown():

        # check we're in the right state and have a Target
        if current_state == 4 and not target == null_target:
            '''
            Get First Probe Point
            '''
            if probe_plan_state == 0:

                if not set_desired_gantry_pose:
                    # define desired probe tip position in gantry frame
                    desired_probe_tip.x = target.x - landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.y = target.y
                    desired_probe_tip.z = -scorpion_gantry_offset_loc[2] + landmine_pos[2] # set to depth
                    gantry_yaw = 0

                    move_gantry(desired_probe_tip, gantry_yaw)
                    set_probe = False

                elif not set_probe and gantry_current_state[6] == 1: # we're finished moving the gantry
                    probe_cmd_pub.publish(2) # start probing
                    set_probe = True

                elif set_desired_gantry_pose and set_probe and probe_current_state[2] == 1: # we're finished probing
                    # generate new plan or change state based on contact point
                    if len(contact_points) > 0:
                        probe_plan_state = 1
                        set_desired_gantry_pose = False
                    else:
                        desired_probe_tip.x += maxForwardSearch
                        move_gantry(desired_probe_tip, gantry_yaw)
                        set_probe = False

            elif probe_plan_state == 1:

                if not set_desired_gantry_pose:

                    # define desired probe tip position in gantry frame
                    th = np.array([-math.pi/2,0,math.pi/2])
                    print "th", th
                    x = landmine_diameter/2*np.cos(th) + contact_points[0,0]
                    print "x", x
                    y = landmine_diameter/2*np.sin(th) + contact_points[0,1]
                    print "y", y
                    z = np.ones(len(th))*contact_points[0,2]
                    print "z", z

                    if target.y > gantry_width/2:
                        gantry_yaw = th[2] # get desired yaw
                        plan_x = x[0]
                        plan_y = y[0]
                    else:
                        gantry_yaw = th[0] # get desired yaw
                        plan_x = x[2]
                        plan_y = y[2]

                    print "gantry_yaw", gantry_yaw

                    desired_probe_tip.x = plan_x # - math.cos(gantry_yaw)*landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.y = plan_y # + math.sin(gantry_yaw)*landmine_diameter/2*probe_safety_factor
                    desired_probe_tip.z = -scorpion_gantry_offset_loc[2] + landmine_pos[2] # set to depth

                    plot_point(desired_probe_tip.x,desired_probe_tip.y,desired_probe_tip.z) # debug

                    print "desired_probe_tip", desired_probe_tip

                    move_gantry(desired_probe_tip, gantry_yaw)
                    set_probe = False

                elif not set_probe and gantry_current_state[6] == 1: # we're finished moving the gantry

                    probe_cmd_pub.publish(2) # start probing
                    set_probe = True

                elif set_desired_gantry_pose and set_probe and probe_current_state[2] == 1: # we're finished probing

                    # generate new plan or change state based on contact point
                    if len(contact_points) > 1:
                        probe_plan_state = 2
                    else:
                        desired_probe_tip.x += math.cos(gantry_yaw)*maxForwardSearch
                        desired_probe_tip.x += math.sin(gantry_yaw)*maxForwardSearch
                        move_gantry(desired_probe_tip, gantry_yaw)
                        set_probe = False

            elif probe_plan_state == 2:

                # TODO
                print "Fill in the gaps"
                pass

        r.sleep()

if __name__ == "__main__":
    main()

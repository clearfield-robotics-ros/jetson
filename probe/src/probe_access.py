# import rospy
import numpy as np
# import tf
import math
# from mine_estimator import Mine_Estimator
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

##############################
### PARAMETERS FOR MINEBOT ###
##############################

# ### Landmine ###
# landmine_pos: [1800, 100, -150]
# landmine_diameter: 100
# landmine_height: 50
# metal_detector_decay: 30

# ### Scorpion Model ###
# model_scorpion_offset_loc: [-450,-425,0]
# model_scorpion_offset_rot: [1.57079632679, 0, 1.57079632679]

# ### Gantry ###
# scorpion_gantry_offset_loc: [1200,-400,300] # y offset is half of width
# scorpion_gantry_offset_rot: [0,0,0]
# gantry_width: 800 #mm
# gantry_sweep_speed: 500 #mm/s
# gantry_x_range: 400 #mm
# gantry_y_range: 700 #mm
# ### Gantry Simulation ###
# gantry_rot_speed: 1.5 #rad/s

### Metal Detector ###
# md_gantry_offset_loc: [350,-100,-300] # y offset is half of width

### Probe ###

# probe_base_offset_rot: [0,0.523599,1.5708] # start out of the way
# probe_offset_distance: 350
# probe_speed: 1.5 #mm/s
# probe_safety_factor: 1
# max_probe_distance: 400

### Probe Planner ###
# third_stage_probe_angle: 1.3962634016
# num_contact_points: 5
# min_fit_error: 2.
probe_angle = math.pi/6
gantry_x_range = 400 # mm   FOR NOW ONLY ASSUME WE'RE IN THE PROJECTED BOUNDS OF GANTRY
gantry_y_range = 800 # mm
probe_base_offset_loc = [0,100,-100]
probe_length = 400
nx, ny, nth = (5, 9, 360)
yaw_range = math.pi*2
x = np.linspace(0,gantry_x_range, nx)
y = np.linspace(0,gantry_y_range, ny)
t = np.linspace(0,yaw_range, nth)
np.set_printoptions(precision=2)

class loc_data:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = -150

def main():
    xv, yv = np.meshgrid(x, y, sparse = False, indexing = 'ij')
    # print xv
    loc = loc_data()
    summ = np.empty([nx,ny])
    initial_angle = np.empty([nx,ny])
    for i in range(nx):
        loc.x = x[i]
        for j in range(ny):
            loc.y = y[j]
            pose_possible = []
            for k in range(nth):
                pose_possible.append(within_gantry_limits(loc,t[k]))
            initial_angle[i,j] = calc_initial_approach_angle(pose_possible)
            summ[i,j] = calc_accessible_angle_range(pose_possible)
    print "Range of approach angles\n", summ
    print "Initial approach angle\n", initial_angle

def calc_initial_approach_angle(possible):
    prod = 0
    for i in range(len(possible)):
        # print i*possible[i]*360/nth
        prod += i*possible[i]*360/nth
        sum = np.sum(possible).astype(float)
    # print prod, sum
    angle = prod/sum
    # raw_input()
    return angle 

def calc_accessible_angle_range(possible):
    return np.sum(possible).astype(float)*(360/nth)

def within_gantry_limits(loc, yaw):
    pose_possible = False # start by assuming it's out of bounds 
    candidate_trans = probe_to_gantry_transform(loc, yaw) # get transform for specified loc & yaw
    if candidate_trans[0] >= 0 and candidate_trans[0] <= gantry_x_range: # if within both x bounds
        if candidate_trans[1] >= 0 and candidate_trans[1] <= gantry_y_range: # and y bounds
            pose_possible = True # switch the flag
    return pose_possible

# def initial_approach_angle(loc_estimate):


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

if __name__ == "__main__":
    main()





# def calc_yaw_angle_from_center(loc):
#   gantry_x_center = loc.x/2
#   gantry_y_center = loc.y/2
#   theta = Math.atan2(loc.y-gantry_y_center, loc.x-gantry_x_center)

#   return theta
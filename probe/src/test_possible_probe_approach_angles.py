from constraints_redo import Probe_Motion_Planner
from shapely.geometry import Point as shapely_Point

##############################
### PARAMETERS FOR MINEBOT ###
##############################

### Landmine ###

landmine_pos = [1800, -50, -100]
landmine_diameter = 100
landmine_height = 50
metal_detector_decay = 30

### Scorpion Model ###
model_scorpion_offset_loc = [-450,-425,0]
model_scorpion_offset_rot = [1.57079632679, 0, 1.57079632679]
### Gantry Simulation ###
sim_walking_speed = 1.0 # set to zero when testing!

### Gantry ###
scorpion_gantry_offset_loc = [1200,-400,493] # y offset is half of width
scorpion_gantry_offset_rot = [0,0,0]
gantry_width = 800 #mm
gantry_sweep_speed = 70 # %

### Gantry Simulation ###
gantry_rot_speed = 1.5 # 1.5 #rad/s

### Metal Detector ###
sensorhead_md_offset_loc = [110, 345, -455] #[140,373,-455] # y offset is half of width
sensorhead_md_offset_rot = [0,0,1.5708] # y offset is half of width
gantry_sweep_angle = -1.09956 # -63 degrees
gantry_sweep_x_pos = 50

### Probe ###
sensorhead_probebase_offset_loc = [-289.51,0,-223.9]
sensorhead_probebase_offset_rot = [0,0.523599,0]
probe_length = 460 #mm
probe_speed = 1.5 #mm/s
probe_safety_factor = 1
max_probe_distance = 300 #mm
max_num_probes = 3
calibrate_probe = False

### Probe Planner ###
third_stage_probe_angle = 1.57079632679 #1.3962634016
num_contact_points = 5
min_fit_error = 2.
gantry_x_min = 0
gantry_x_max = 400
gantry_y_min = 0
gantry_y_max = 800
gantry_th_min = -1.57079632679
gantry_th_max = 1.57079632679
gantry_ticks_per_mm = 5.61121

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

def calc_probe_angle_range(desired_probe_tip):
    # gantry_th_min                  = rospy.get_param('gantry_th_min')
    # gantry_th_max                  = rospy.get_param('gantry_th_max')
    possible_probe_approach_angles = np.linspace(gantry_th_min, gantry_th_max, 19)

    probe_motion_planner = Probe_Motion_Planner([0,0,0], [0,0,0]) # just instantiate with dummy points

    collision_free_points = []
    for angle in possible_probe_approach_angles:
        trans = probe_to_gantry_transform(desired_probe_tip, angle)
        point_to_check = shapely_Point(trans[1]/1000.0, angle) # Point takes (y[mm], th[rad])
        collision_free_points.append(probe_motion_planner.point_collision_free(point_to_check))
    print collision_free_points

def get_desired_probe_tip(index):


def main():
	calc_probe_angle_range()

if __name__ == "__main__":
    main()

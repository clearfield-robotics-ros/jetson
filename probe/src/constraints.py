from shapely.geometry import Polygon, Point, LineString
import shapely
import math
import time
import numpy as np
import networkx as nx
from matplotlib import pyplot as plt
from shapely.geometry.polygon import Polygon
import rospy

# KEY ASSUMPTIONS
# the gantry mean is along the vehicle center axis


# do_plot = False

# fig = plt.figure(1, figsize=(5,5), dpi=90)
# ax = fig.add_subplot(111)

class Probe_Motion_Planner:
    def __init__(self, start, end):
        gantry_y_min                    = rospy.get_param('gantry_y_min')/1000 # mm to m
        gantry_y_max                    = rospy.get_param('gantry_y_max')/1000 # mm to m
        gantry_th_min                   = rospy.get_param('gantry_th_min')
        gantry_th_max                   = rospy.get_param('gantry_th_max')
        self.gantry_y_limits = [gantry_y_min, gantry_y_max] # in m so that the scales of y and th are similar
        self.gantry_th_limits = [gantry_th_min, gantry_th_max]
        gantry_y_mean = (self.gantry_y_limits[0] + self.gantry_y_limits[1])/2
        self.off_limits = [[(gantry_y_mean+.220, np.deg2rad(-45)), (self.gantry_y_limits[1], np.deg2rad(-45)), (self.gantry_y_limits[1], np.deg2rad(30)), (gantry_y_mean+.220, np.deg2rad(30))],
                          [(gantry_y_mean+.270, np.deg2rad(80)), (self.gantry_y_limits[1], np.deg2rad(80)), (self.gantry_y_limits[1], np.deg2rad(90)), (gantry_y_mean+.270, np.deg2rad(90))],
                          [(gantry_y_mean-.270, np.deg2rad(-90)), (self.gantry_y_limits[0], np.deg2rad(-90)), (self.gantry_y_limits[0], np.deg2rad(-75)), (gantry_y_mean-.270, np.deg2rad(-75))]]
        self.max_extend_dist = 0.5 # max radius to extend each node from
        # start_config = [.40, .75, -1.5]
        # end_config = [.40, .750, .65]
        # config[0] (X axis) not a concern for any configuration
        # first check whether the end_config is
        self.start_point = Point(start[1], start[2])
        self.end_point = Point(end[1], end[2])

    def check_collision(self, point):
        in_collision = [] # create empty array to store collision results
        for zone in range(len(self.off_limits)): # go thorugh each off-limits zone # I KNOW THIS IS TERRIBLE PYTHON STYLE >_<
            poly = Polygon(self.off_limits[zone]) # create a polygon for that zone
            in_collision.append(point.within(poly)) # add to the list whether or not it collided
        if np.any(in_collision):
            return True
        else: 
            return False

    def check_ray_collision(self, line):
        in_collision = [] # create empty array to store collision results
        for zone in range(len(self.off_limits)): # go thorugh each off-limits zone # I KNOW THIS IS TERRIBLE PYTHON STYLE >_<
            poly = Polygon(self.off_limits[zone]) # create a polygon for that zone
            in_collision.append(line.intersects(poly)) # add to the list whether or not it collided
        if np.any(in_collision):
            return True
        else: 
            return False

    def plan_path(self):
        start = self.start_point
        end = self.end_point
        goal_reached = False # start by assuming you're not at the goal, DUH!
        max_dist = self.max_extend_dist 
        visited_points = [start]
        G = nx.Graph() # nx is a great way to represent graph connections
        q_new_index = 0
        G.add_node(q_new_index)
        while not goal_reached:
            q_rand = self.sample_random_point()
            nearest_q_candidate_dist = []
            for i in range(len(visited_points)):
                nearest_q_candidate_dist.append(q_rand.distance(visited_points[i])) # for every visited point, store the distance to the randomly sampled point
            q_near_index = np.argmin(nearest_q_candidate_dist) # choose the point with the lowest distance among those visited
            q_near = visited_points[q_near_index] # get the actual point (not index)
            q_new, goal_reached = self.extend(q_near, q_rand, end, max_dist)
            # x, y = q_new.xy
            if not self.check_collision(q_new):
                q_new_index += 1
                # if do_plot:
                #     ax.scatter(x, y, c='k')
                #     plt.pause(0.01)
                visited_points.append(q_new)
                G.add_node(q_new_index)
                G.add_edge(q_near_index, q_new_index)
                if goal_reached:
                    # if do_plot:
                    #     ax.scatter(x, y, c='g')
                    path_sequence = nx.shortest_path(G, source=0, target=q_new_index)
                    path_points = [visited_points[i] for i in path_sequence]
                    shorter_path = self.shorten_path(path_points)
            # if self.check_collision(q_new):
            #     if do_plot:
            #         ax.scatter(x, y, c='r')
            #         plt.pause(0.01)
        # if do_plot:
        #     for i in range(len(shorter_path)-1):
        #         pt1 = shorter_path[i]
        #         pt2 = shorter_path[i+1]
        #         ax.plot([pt1.x, pt2.x],[pt1.y, pt2.y], c='k')
        #     plt.show()
        return shorter_path


    def shorten_path(self, path, timeout=0.1):
                
        start_time = time.time() # mark current time
        num_sample_edges = 2
        iteration = 0

        while (time.time()-start_time)<timeout: # until timeout expires
            num_edges_old = len(path)-1
            sample_pt_array = []
            sample_edge = np.random.choice(np.arange(num_edges_old),2,replace=False)
            for edge in range(num_sample_edges): # for each pair of points
                ratio = np.random.random_sample() # sample in [0,1)
                pt1 = path[sample_edge[edge]] # first vertex of a given edge
                pt2 = path[sample_edge[edge]+1] # second vertex of a given edge
                pt1_pt2 = LineString([pt1, pt2]) # line joining two
                sample_pt_array.append(pt1_pt2.interpolate(ratio, normalized=True))
            interp_pt1 = sample_pt_array[0] # extract sample point 1 coordinates
            interp_pt2 = sample_pt_array[1] # extract sample point 2 coordinates
            if not self.check_ray_collision(LineString([interp_pt1, interp_pt2])):
                first_edge = sample_edge[0]
                second_edge = sample_edge[1]
                ascending = second_edge>first_edge
                if ascending:
                    new_pair = [interp_pt1, interp_pt2]
                    rows_to_delete = range(first_edge+1,second_edge+1)
                else:
                    new_pair = [interp_pt2, interp_pt1]
                    rows_to_delete = range(second_edge+1,first_edge+1)
                path = [x for i,x in enumerate(path) if i not in rows_to_delete]
                path.insert(np.min(rows_to_delete),new_pair[0]) # replace with shortened path rows
                path.insert(np.min(rows_to_delete)+1,new_pair[1])
        return path


    def sample_random_point(self):
        y = (self.gantry_y_limits[1] - self.gantry_y_limits[0]) * np.random.random_sample() + self.gantry_y_limits[0]  
        th = (self.gantry_th_limits[1] - self.gantry_th_limits[0]) * np.random.random_sample() + self.gantry_th_limits[0]  
        return Point(y, th)

    def extend(self, current, next, goal, max_dist):
        if current.distance(goal) <= max_dist: # if you're within striking distance of the goal config
            return goal, True
        else:
            if current.distance(next) <= max_dist: # if next node is right next to you
                return next, False
            else: # if it's out of reach, interpolate as far as you can go
                current_next = LineString([current, next]) # line joining two
                return current_next.interpolate(max_dist), False

# def main():
#     start_config = [.40, .75, -1.5]
#     end_config = [.40, .750, .65]
#     # config[0] (X axis) not a concern for any configuration
#     # first check whether the end_config is
#     start_point = Point(start_config[1], start_config[2])
#     end_point = Point(end_config[1], end_config[2])

#     if check_collision(end_point):
#         print "Goal is out of bounds"
#     else:
#         plan_path(start_point, end_point)

# if __name__ == "__main__":
#     main()
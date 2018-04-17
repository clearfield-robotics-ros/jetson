from shapely.geometry import Polygon, Point, LineString
import shapely
import math
import time
import numpy as np
import networkx as nx
from matplotlib import pyplot as plt
from shapely.geometry.polygon import Polygon
import rospy

do_plot = True
do_path_shortening = False

fig = plt.figure(1, figsize=(6, 6), dpi=90)
ax = fig.add_subplot(111)
ax.set_xlabel('Gantry Y position (m)')
ax.set_ylabel('Yaw angle (rad)')
ax.set_title('RRT Planner for Gantry Pos')

class Probe_Motion_Planner:
    def __init__(self, start, end):
        """
        Creates a Probe_Motion_Planner object
        """
        # Gantry limit parameters
        self.gantry_y_min                    = rospy.get_param('gantry_y_min')/1000.0 # mm to m
        self.gantry_y_max                    = rospy.get_param('gantry_y_max')/1000.0 # mm to m
        self.gantry_th_min                   = rospy.get_param('gantry_th_min') # (rad) CCW positive, 0 is probe facing forward
        self.gantry_th_max                   = rospy.get_param('gantry_th_max') # (rad)

        gantry_y_mean = (self.gantry_y_min + self.gantry_y_max)/2

        # Obstacle definitions within gantry limits
        self.off_limits = [[(gantry_y_mean+.220, np.deg2rad(45)), (self.gantry_y_max, np.deg2rad(45)), (self.gantry_y_max, np.deg2rad(-30)), (gantry_y_mean+.220, np.deg2rad(-30))],
                           [(gantry_y_mean+.270, np.deg2rad(-80)), (self.gantry_y_max, np.deg2rad(-80)), (self.gantry_y_max, np.deg2rad(-90)), (gantry_y_mean+.270, np.deg2rad(-90))],
                           [(gantry_y_mean-.270, np.deg2rad(90)), (self.gantry_y_min, np.deg2rad(90)), (self.gantry_y_min, np.deg2rad(75)), (gantry_y_mean-.270, np.deg2rad(75))]]

        self.max_extend_dist = 1 # max radius to extend each node

        # Convert start and end points to local Point format
        self.start_point = self.config_mm_to_point_m(start)
        self.end_point = self.config_mm_to_point_m(end) 

    def point_collision_free(self, point):
        """
        Checks whether a given point is within any obstacle polygons OR if it is out of bounds

        :param line: Point object
        :return: returns True if it IS collision free, False if it is NOT collision free
        """  
        in_collision = [] # create empty array to store collision results
        for zone in range(len(self.off_limits)): # first, go through each off-limits zone
            poly = Polygon(self.off_limits[zone]) # create a polygon for that zone
            in_collision.append(point.within(poly)) # add to the list whether or not it collided
            if do_plot:
                x, y = poly.exterior.xy
                ax.plot(x, y, color='#6699cc', alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2)
                plt.pause(0.01)

        test_point_y = point.x # create test points for boundary checking
        test_point_th = point.y

        if test_point_y >= self.gantry_y_max or test_point_y <= self.gantry_y_min \
            or test_point_th >= self.gantry_th_max or test_point_th <= self.gantry_th_min: # second, now check if it's outside the boundaries
            in_collision.append(True)
            if do_plot:
                ax.scatter(x, y, c='r')
                plt.pause(0.01)

        if np.any(in_collision):
            return False # if it IS in collision with anything, it is NOT collision free
        else:
            return True # if it is NOT in collision with anything, it IS collision free


    def line_collision_free(self, line):
        """
        Checks whether a given line intersects any obstacle polygons

        :param line: LineString object
        :return: returns True if it IS collision free, False if it is NOT collision free
        """        

        in_collision = [] # create empty array to store collision results
        for zone in range(len(self.off_limits)): # go thorugh each off-limits zone # I KNOW THIS IS TERRIBLE PYTHON STYLE >_<
            poly = Polygon(self.off_limits[zone]) # create a polygon for that zone
            if do_plot:
                x, y = poly.exterior.xy
                ax.plot(x, y, color='#6699cc', alpha=0.7, linewidth=3, solid_capstyle='round', zorder=2)
                plt.pause(0.01)
            in_collision.append(line.intersects(poly)) # add to the list whether or not it collided

        if np.any(in_collision):
            return False # if it IS in collision with anything, it is NOT collision free
        else: 
            return True # if it is NOT in collision with anything, it IS collision free


    def plan_path(self):

        # first, check if the end point is valid
        # if it is NOT valid, return the path as just start point to start point (i.e. do nothing)
        if not self.point_collision_free(self.end_point):
            print "End point out of bounds"
            return [self.start_point, self.start_point]

        # second, check if there is a straight line between start and end points
        # if there is NO collision, just return the straight line
        if self.line_collision_free(LineString([self.start_point, self.end_point])):
            print "Straight line joins points"
            if do_plot:
                xs, ys = self.start_point.xy
                xe, ye = self.end_point.xy
                ax.scatter(xs, ys, c='m')
                ax.scatter(xe, ye, c='g')
                plt.pause(0.01)
                plt.show()
            return [self.start_point, self.end_point]

        # if you've made it this far, the end point is valid AND there is an obstacle in the 
        #   way between the start and end points

        goal_reached = False # start by assuming you're not at the goal, DUH!
        valid_visited_points = [self.start_point]
        if do_plot:
            x, y = self.start_point.xy
            ax.scatter(x, y, c='m')
            plt.pause(0.01)
        G = nx.Graph() # nx is a great way to represent graph connections
        q_new_index = 0
        G.add_node(q_new_index)
        while not goal_reached:
            q_rand = self.sample_random_point()
            nearest_q_candidate_dist = []
            for i in range(len(valid_visited_points)):
                nearest_q_candidate_dist.append(q_rand.distance(valid_visited_points[i])) # for every valid visited point, store the distance to the randomly sampled point
            q_near_index = np.argmin(nearest_q_candidate_dist) # choose the point with the lowest distance among those visited
            q_near = valid_visited_points[q_near_index] # get the actual point (not index)
            q_new, goal_reached = self.extend(q_near, q_rand, self.end_point, self.max_extend_dist)
            x, y = q_new.xy
            if self.line_collision_free(LineString([q_near, q_new])): # if the line joining the existing point and new candidate point is collision-free, add it to the tree
                q_new_index += 1
                if do_plot:
                    ax.scatter(x, y, c='k')
                    plt.pause(0.01)
                valid_visited_points.append(q_new)
                G.add_node(q_new_index)
                G.add_edge(q_near_index, q_new_index)
                if goal_reached:
                    if do_plot:
                        ax.scatter(x, y, c='g')
                    path_sequence = nx.shortest_path(G, source=0, target=q_new_index) # find shortest path using nx utility
                    path_points = [valid_visited_points[i] for i in path_sequence]
            else:
                goal_reached = False
                if do_plot:
                    ax.scatter(x, y, c='r')
        print "here"
        if do_path_shortening:
            path_points = self.shorten_path(path_points)
        if do_plot: # plot the path from start to end
            for i in range(len(path_points)-1):
                pt1 = path_points[i]
                pt2 = path_points[i+1]
                ax.plot([pt1.x, pt2.x],[pt1.y, pt2.y], c='k')
                plt.pause(0.01)
            plt.show()
        return path_points

    def shorten_path(self, path, timeout=0.5):
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
                # if do_plot:
                #     ax.scatter(pt1.x, pt1.y, c='y')
                #     ax.scatter(pt2.x, pt2.y, c='r')
                pt1_pt2 = LineString([pt1, pt2]) # line joining two
                sample_pt_array.append(pt1_pt2.interpolate(ratio, normalized=True))
            interp_pt1 = sample_pt_array[0] # extract sample point 1 coordinates
            interp_pt2 = sample_pt_array[1] # extract sample point 2 coordinates
            if self.line_collision_free(LineString([interp_pt1, interp_pt2])):
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
        """
        Creates a random combination of y and th

        :return: returns a Point(y (m), th (rad))
        """        
        y = (self.gantry_y_max - self.gantry_y_min) * np.random.random_sample() + self.gantry_y_min
        th = (self.gantry_th_max - self.gantry_th_min) * np.random.random_sample() + self.gantry_th_min
        return Point(y, th)

    def extend(self, current, next, goal, max_dist):
        """
        Generates a valid point which can be added to the RRT tree

        :param current: current point (Point object)
        :param next: candidate for next point (Point object)
        :param goal: the goal point (Point object)
        :param max_dist: max radius next point can lie within
        :return next_point: returns the next valid point (Point object)
        :return goal_reached: returns True if the next point is the goal, False otherwise
        """

        if current.distance(goal) <= max_dist: # if you're within striking distance of the goal config
            next_point = goal
            goal_reached = True
        else:
            if current.distance(next) <= max_dist: # if next node is right next to you
                next_point = next
                goal_reached = False
            else: # if it's out of reach, interpolate as far as you can go
                current_next = LineString([current, next]) # make a line joining two
                next_point = current_next.interpolate(max_dist)
                goal_reached = False

        return next_point, goal_reached

    def config_mm_to_point_m(self, config):
        """
        Converts an input config list to a Shapely Point object type

        :param config: list of config [x (mm), y (mm), th (rad)]
        :return: returns a Point(y (m), th (rad))
        """
        return Point(config[1]/1000.0, config[2])       
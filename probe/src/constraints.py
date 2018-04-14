from shapely.geometry import Polygon, Point, LineString
import shapely
import math
import numpy as np
import networkx as nx
from matplotlib import pyplot as plt
from shapely.geometry.polygon import LinearRing, Polygon

GANTRY_Y_LIMITS = [0, .800]
GANTRY_TH_LIMITS = [np.deg2rad(-90), np.deg2rad(90)]
gantry_y_mean = (GANTRY_Y_LIMITS[0] + GANTRY_Y_LIMITS[1])/2
off_limits = [[(gantry_y_mean+.220, np.deg2rad(-45)), (GANTRY_Y_LIMITS[1], np.deg2rad(-45)), (GANTRY_Y_LIMITS[1], np.deg2rad(30)), (gantry_y_mean+.220, np.deg2rad(30))],
			  [(gantry_y_mean+.270, np.deg2rad(80)), (GANTRY_Y_LIMITS[1], np.deg2rad(80)), (GANTRY_Y_LIMITS[1], np.deg2rad(90)), (gantry_y_mean+.270, np.deg2rad(90))],
			  [(gantry_y_mean-.270, np.deg2rad(-90)), (GANTRY_Y_LIMITS[0], np.deg2rad(-90)), (GANTRY_Y_LIMITS[0], np.deg2rad(-75)), (gantry_y_mean-.270, np.deg2rad(-75))]]

def check_collision(point):
	in_collision = [] # create empty array to store collision results
	for zone in range(len(off_limits)): # go thorugh each off-limits zone # I KNOW THIS IS TERRIBLE PYTHON STYLE >_<
		poly = Polygon(off_limits[zone]) # create a polygon for that zone
		in_collision.append(point.within(poly)) # add to the list whether or not it collided
		# print "here"
		# x, y = poly.exterior.xy
		# fig = plt.figure(1, figsize=(5,5), dpi=90)
		# ax = fig.add_subplot(111)
		# ax.plot(x, y, color='#6699cc', alpha=0.7,
		#     linewidth=3, solid_capstyle='round', zorder=2)
		# ax.set_title('Polygon')
		# plt.show()
		# raw_input()
	if np.any(in_collision):
		return True
	else: 
		return False

def plan_path(start, end):
	goal_reached = False # start by assuming you're not at the goal, DUH!
	max_dist = 0.5
	visited_points = [start]
	G = nx.Graph()
	q_new_index = 0
	G.add_node(q_new_index)
	while not goal_reached:
		q_new_index += 1
		q_rand = sample_random_point()
		nearest_q_candidate_dist = []
		for i in range(len(visited_points)):
			nearest_q_candidate_dist.append(q_rand.distance(visited_points[i])) # for every visited point, store the distance to the randomly sampled point
		q_near_index = np.argmin(nearest_q_candidate_dist) # choose the point with the lowest distance among those visited
		q_near = visited_points[q_near_index] # get the actual point (not index)
		q_new, goal_reached = extend(q_near, q_rand, end, max_dist)
		# print q_new
		# check if q_new is in collision
		if not check_collision(q_new):
			fig = plt.figure(1, figsize=(5,5), dpi=90)
			ax = fig.add_subplot(111)
			x, y = q_new.xy
			ax.scatter(x, y)
			plt.pause(0.01)
			# plt.show(block=True)
			visited_points.append(q_new)
			G.add_node(q_new_index)
			G.add_edge(q_near_index, q_new_index)
			if goal_reached:
				print nx.predecessor(G, 0, target=q_new_index)
				# print nx.shortest_path(G, source=0, target=q_new_index)
	plt.show()
	for i in range(len(visited_points)):
		print visited_points[i].xy

def sample_random_point():
	y = (GANTRY_Y_LIMITS[1] - GANTRY_Y_LIMITS[0]) * np.random.random_sample() + GANTRY_Y_LIMITS[0]	
	th = (GANTRY_TH_LIMITS[1] - GANTRY_TH_LIMITS[0]) * np.random.random_sample() + GANTRY_TH_LIMITS[0]	
	return Point(y, th)

def extend(current, next, goal, max_dist):
	# print next
	if current.distance(goal) <= max_dist: # if you're within striking distance of the goal config
		return goal, True
	else:
		if current.distance(next) <= max_dist: # if next node is right next to you
			return next, False
		else: # if it's out of reach, interpolate as far as you can go
			current_next = LineString([current, next]) # line joining two
			return current_next.interpolate(max_dist), False

def main():
	start_config = [.40, .650, -1.5]
	end_config = [.40, .650, .78]
	# config[0] (X axis) not a concern for any configuration
	# first check whether the end_config is
	start_point = Point(start_config[1], start_config[2])
	end_point = Point(end_config[1], end_config[2])
	# print check_gantry_collision(end_point)

	if check_collision(end_point):
		print "Goal is out of bounds"
	else:
		plan_path(start_point, end_point)

	# # test extend - working fine
	# next_point, done = extend(start_point, Point(600, 0.-1.4), end_point, 0.5)
	# print next_point, done

if __name__ == "__main__":
    main()
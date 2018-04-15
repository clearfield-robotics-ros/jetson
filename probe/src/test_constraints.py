#!/usr/bin/env python

from constraints import Probe_Motion_Planner
from shapely.geometry import Polygon, Point, LineString
import shapely

def main():
    print "here"
    start_config = [100, 780, 0.84]
    end_config = [100, 780, -0.84]
    probe_motion_planner = Probe_Motion_Planner(start_config, end_config)
    end_config_valid = probe_motion_planner.end_point_valid()
    if end_config_valid:
        path = probe_motion_planner.plan_path()
        plan_arrays = [[step.x, step.y] for step in path] # reconfigure into an array of arrays
        print plan_arrays
    else: 
        path = []
        plan_arrays = []
    print plan_arrays

if __name__ == "__main__":
    main()

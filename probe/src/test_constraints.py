#!/usr/bin/env python

from constraints import Probe_Motion_Planner
from shapely.geometry import Polygon, Point, LineString
import shapely

def main():
	start_config = [.40, .75, -1.5]
	end_config = [.40, .750, .7]
	probe_motion_planner = Probe_Motion_Planner(start_config, end_config)
	path = probe_motion_planner.plan_path()
	print path

if __name__ == "__main__":
    main()

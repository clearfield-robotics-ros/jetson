#!/usr/bin/env python

from constraints import Probe_Motion_Planner

def main():
	start_config = [.40, .75, -1.5]
	end_config = [.40, .750, .65]
	probe_motion_planner = Probe_Motion_Planner(start_config, end_config)
	print probe_motion_planner.plan_path()

if __name__ == "__main__":
    main()

from mine_estimator_non_ros import Mine_Estimator
import math
import numpy as np

x_offset = 0#500
y_offset = 0#500
def main():

	print "\n\n\n"
	diam = 100
	height = 50
	z = -100
	deg2rad = math.pi/180
	est = Mine_Estimator(diam, height)

	# make first contact point
	x, y = polar_to_cartesian(est.radius, 0)
	est.add_point(x,y,z)

	# make second contact point
	x, y = polar_to_cartesian(est.radius, 15*deg2rad)
	est.add_point(x,y,z)

	# make third contact point
	x, y = polar_to_cartesian(est.radius, 30*deg2rad)
	est.add_point(x,y,z)

	print est.contact_points

	print est.get_sparsest_point()

def polar_to_cartesian(r, th):
	x = -math.cos(th) * r + x_offset# + 2*np.random.random()-1
	y = -math.sin(th) * r + y_offset# + 2*np.random.random()-1

	return x, y

if __name__ == "__main__":
    main()
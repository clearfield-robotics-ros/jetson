import math 
import numpy as np
import itertools

class pos():
    def __init__(self):
        self.x = 0.
        self.y = 0.
        self.z = 0.


class Calculation():


    def __init__(self):
        self.radius = 50 # mm
        self.d2r = math.pi/180
        self.pts = np.array([[0,0,0],[35.7, 35.2, 0],[-34.9, 36.1, 0]])
    #     r, th = generate_three_fake_noisy_pts


    # def generate_three_fake_noisy_pts(self):
    #     th_noise = (2*np.random.random([1,3])-1).squeeze()*5.*d2r
    #     th = np.array([0.,45.,-45.])*d2r+th_noise
    #     r_noise = (2*np.random.random([1,3])-1).squeeze()*5.
    #     r = np.full((1,3), radius).squeeze()
    #     r = r + r_noise
    #     return r, th

    def get_sparsest_point(self):
        combinations = [x for x in self.get_pair_combinations(self.pts)]
        print combinations
        for i in combinations:
            print np.linalg.norm(self.pts[i[1]]-self.pts[i[0]])

    def get_pair_combinations(self, points):
        return itertools.combinations(range(len(points)), 2)

def main():
    calc = Calculation()
    calc.get_sparsest_point()
    # orig_center_estimate = pos()
    # r, th = generate_three_fake_noisy_pts()

    # get_sparsest_point(orig_center_estimate)

if __name__ == "__main__":
    main()

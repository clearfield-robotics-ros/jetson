import math

class Mine_Ground_Truth_Locations:
    def __init__(self):
        """
        Creates a Mine_Ground_Truth_Locations object
        """
        self.l = 5.00 # length of one edge in meters
        self.d = [[3.25, 3.08], # 
                  [2.74, 2.90], #       *      *            ||| TRAILER THIS CORNER
                  [2.74, 2.90], #
                  [2.74, 2.90], #          #                # = target
                  [2.74, 2.90], #   d[0]  /  \  d[1]        d[0] and d[1] measured from corners
                  [2.74, 2.90], #        /    \  
                  [2.74, 2.90], #       *______*            ||| NREC BUILDING THIS CORNER
                  [2.74, 2.90], #           l
                  [2.74, 2.90], # 
                  [2.74, 2.90]] # 
        self.locations = self._calc_all_locations()
        self.ground_truth_order = [4,2,7,6,9,0,3,1,8,5] # change the indices corresponding to the test sequence
        self.current_index = 0

    def _calc_all_locations(self):
        loc = []
        for pos in self.d:
            th = math.acos((pos[0]**2 + self.l**2 - pos[1]**2)/(2*pos[0]*self.l))
            loc.append([pos[0]*math.cos(th), pos[0]*math.sin(th)])
        return loc

    def get_ground_truth_loc(self):
        ground_truth_loc = self.locations[self.current_index]
        self._increment_index()
        return ground_truth_loc

    def _increment_index(self):
        self.current_index += 1 # manages incrementing of which target you're interested in

def main():
    loc = Mine_Ground_Truth_Locations()
    for i in range(4):
        latest_loc = loc.get_ground_truth_loc()
        print latest_loc

if __name__ == "__main__":
    main()

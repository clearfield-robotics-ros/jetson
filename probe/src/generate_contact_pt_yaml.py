#!/usr/bin/env python

from mine_estimator import Mine_Estimator
import datetime
import os

class Generate_Contact_Point_Yaml():
    def __init__(self, timestamp):
        cwd = os.getcwd()
        self.filename = str(cwd) + "/contact_points_" + str(timestamp.month) + "_" + str(timestamp.day) \
            + "_" + str(timestamp.hour) + "_" + str(timestamp.minute) + ".yaml"
        print "Saving to: ", self.filename

    def write_to_file(self, estimates, num_attempted_probes):
        self.estimates = estimates
        with open(self.filename,"w+") as f:
            f.write('test_data:\n')
            for est in self.estimates:
                f.write('- class: ' + str(est.get_result()) + '\n  attempted: ' + str(num_attempted_probes))
                f.write('\n  points: [')
                for i, pt in enumerate(est.contact_points):
                    for elm in range(3):
                        if elm == 2:
                            f.write(str(pt[elm]) + ']')
                        elif elm == 0:
                            f.write(str('[' + str(pt[elm]) + ','))
                        else:
                            f.write(str(pt[elm]) + ',')
                    if i < len(est.contact_points)-1:
                        f.write(',\n')
                    else:
                        f.write(']')
                f.write('\n')
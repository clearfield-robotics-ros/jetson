import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Int16, Int16MultiArray
import numpy as np
from circle_intersection import Geometry
from visualization_msgs.msg import Marker
from sklearn.cluster import KMeans
import itertools
from random import shuffle
import math
import pdb

class Mine_Estimator:
    def __init__(self, diameter, height):
        self.diameter = diameter
        self.radius = diameter/2
        self.height = height
        self.contact_points = np.array([], dtype=np.int64).reshape(0,3)
        self.g = Geometry()

        self.c_x = 0.
        self.c_y = 0.
        self.c_z = 0.
        self.c_r = self.radius
        self.error = 0
        self.num_inliers = 0
        self.num_attempted_probes = 0

        self.visualize = True
        self.contact_viz_id = 0
        self.contact_viz_pub = rospy.Publisher('probe_contact_viz', Marker, queue_size=10)

        self.classification_error_thresh = rospy.get_param('classification_error_thresh')
        self.RANSAC_num_points = rospy.get_param('RANSAC_num_points')
        self.RANSAC_max_iterations = rospy.get_param('RANSAC_max_iterations')
        self.RANSAC_inlier_thresh = rospy.get_param('RANSAC_inlier_thresh')


    def draw_radius(self,col):

        msg = Marker()
        msg.header.frame_id = "gantry"
        msg.id = 1
        msg.header.seq = 1
        msg.header.stamp = rospy.Time.now()
        msg.ns = "probe_radius_viz"
        msg.type = msg.LINE_STRIP  # line strip
        msg.action = msg.ADD  # add

        msg.pose.orientation.w = 1
        msg.scale.x = 2.
        msg.scale.y = 2.
        msg.scale.z = 2.
        msg.color.a = 1.
        msg.color.r = col[0]
        msg.color.g = col[1]
        msg.color.b = col[2]

        msg.points = []
        for i in range(0,21):

            x = self.radius*math.sin(360/20*i * math.pi/180) + self.c_x
            y = self.radius*math.cos(360/20*i * math.pi/180) + self.c_y
            z = self.c_z
            msg.points.append(Point(x,y,z))

        self.contact_viz_pub.publish(msg)


    def plot_point(self,x,y,z,col):
        # Visualize probe point
        global contact_viz_id
        global contact_viz_pub
        msg = Marker()
        msg.header.frame_id = "gantry"
        # msg.header.frame_id = "probe_tip" # DEBUG
        msg.header.seq = self.contact_viz_id
        msg.header.stamp = rospy.Time.now()
        msg.ns = "probe_contact_viz"
        msg.id = self.contact_viz_id
        msg.type = 2  # cube
        msg.action = 0  # add
        msg.pose.position = Point(x,y,z)
        msg.pose.orientation.w = 1
        msg.scale.x = 10
        msg.scale.y = 10
        msg.scale.z = 10
        msg.color.a = 1.0
        msg.color.r = col[0]
        msg.color.g = col[1]
        msg.color.b = col[2]
        self.contact_viz_pub.publish(msg)
        self.contact_viz_id += 1


    def plot_intersections(self, intersection):
        for i in range(0, len(intersection)):

            x = intersection[i,0]
            y = intersection[i,1]
            z = 0
            msg = Marker()
            msg.header.frame_id = "gantry"
            # msg.header.frame_id = "probe_tip" # DEBUG
            msg.header.seq = i
            msg.id = i
            msg.header.stamp = rospy.Time.now()
            msg.ns = "probe_intersection_viz"
            msg.type = 2  # cube
            msg.action = 0  # add
            msg.pose.position = Point(x,y,z)
            msg.pose.orientation.w = 1
            msg.scale.x = 10
            msg.scale.y = 10
            msg.scale.z = 10
            msg.color.a = 1.0
            msg.color.r = 0.
            msg.color.g = 1.
            msg.color.b = 0.
            self.contact_viz_pub.publish(msg)


    def clear_markers(self):
        msg = Marker()
        msg.ns = "probe_intersection_viz"
        msg.action = msg.DELETEALL
        self.contact_viz_pub.publish(msg)
        msg = Marker()
        msg.ns = "probe_contact_viz"
        msg.action = msg.DELETEALL
        self.contact_viz_pub.publish(msg)
        msg = Marker()
        msg.ns = "probe_radius_viz"
        msg.action = msg.DELETEALL
        self.contact_viz_pub.publish(msg)


    def hough(self, points):
        ### get Circular Intersections and plot
        intersection = np.array([], dtype=np.int64).reshape(0,2)
        for i in range(0,len(points)):
            for j in range(0,i):
                p = self.g.circle_intersection( np.append(points[i,0:2],self.radius),
                                                np.append(points[j,0:2],self.radius) )
                if p is not None:
                    for j in range(0,len(p)):
                        intersection = np.vstack(( intersection,np.array([p[j][0],p[j][1]]) ))

        if len(intersection) > 0: # if we have intersections!

            ### Remove points (bounding box)
            intersection_BB = np.array([], dtype=np.int64).reshape(0,2)
            for i in range(0,len(intersection)):
                if ( intersection[i,0] > min(points[:,0]) ): # and
                # intersection[i,1] > min(points[:,1]) and
                # intersection[i,1] < max(points[:,1]) ):
                    intersection_BB = np.vstack((intersection_BB,intersection[i,:]))
            intersection_BB = np.unique(intersection_BB, axis=0)

            ### Debugging
            if False:
                self.plot_intersections(intersection_BB)

            ### Determine center point
            if len(intersection_BB) > 1:
                km = KMeans(n_clusters=2)
                km.fit(intersection_BB)
                labels = km.labels_
                c_x = km.cluster_centers_[0,0]
                c_y = km.cluster_centers_[0,1]
            elif len(intersection_BB) == 1:
                c_x = intersection_BB[0,0]
                c_y = intersection_BB[0,1]

            return c_x, c_y


    def point_dist(self, p1, p2):
        return abs( math.sqrt( (p1[0]-p2[0])**2+(p1[1]-p2[1])**2 ) - self.c_r )


    def compute_error(self, center, points):
        if len(points) > 0:
            error = 0
            for i in range(0,len(points)):
                error += self.point_dist(points[i,0:2], center)
            return error / len(points) # normalize by number of points


    def circle_fit(self):
        # if self.visualize:
        #     for i in range(0,len(self.contact_points)):
        #         self.plot_point(self.contact_points[i][0],self.contact_points[i][1],self.contact_points[i][2], [1,0,0])

        if len(self.contact_points) == 1:
            ### Just use first contact point for centre point
            self.c_x = self.contact_points[0,0] + self.radius
            self.c_y = self.contact_points[0,1]
            self.error = self.compute_error([self.c_x,self.c_y], self.contact_points)
            self.num_inliers = 1
        else:
            try:
                ### Find combination of inliers that give lowest error circle fit
                result = []
                error = []
                dists = []
                num_inliers = []
                combinations = list(itertools.combinations(self.contact_points, self.RANSAC_num_points))
                shuffle(combinations)
                for i in range(0, min(len(combinations), self.RANSAC_max_iterations) ): # limit to 50 iterations
                    points = np.array(combinations[i])

                    ### compute hough transform
                    c_x, c_y = self.hough(points)
                    result.append([c_x, c_y])

                    ### get the inliers
                    dist = []
                    inlier = []
                    for i in range(0,len(self.contact_points)):
                        dist.append(self.point_dist(self.contact_points[i,0:2], result[-1]))
                        if dist[-1] < self.RANSAC_inlier_thresh:
                            inlier.append([self.contact_points[i,0],self.contact_points[i,1],self.contact_points[i,2]])
                    num_inliers.append(len(inlier))
                    dists.append(dist)

                    ### compute error
                    error.append(self.compute_error(result[-1], np.array(inlier)))

                # Set default idx to max inliers
                max_inliers = max(num_inliers)
                idx = num_inliers.index(max_inliers)

                # let's see if there's any other entries with same max inliers but lower error
                min_error = 1e6
                for i in range(0,len(error)):
                    if num_inliers[i] == max_inliers:
                        if error[i] < min_error:
                            min_error = error[i]
                            idx = i

                # print "num_inliers",num_inliers
                # print "dists",dists[idx]
                # pdb.set_trace()

                self.c_x = result[idx][0]
                self.c_y = result[idx][1]
                self.error = self.compute_error([self.c_x,self.c_y], self.contact_points)
                self.num_inliers = max_inliers

            except:
                self.c_x, self.c_y = self.hough(self.contact_points)
                self.error = self.compute_error([self.c_x,self.c_y], self.contact_points)
                self.num_inliers = len(self.contact_points)

        if self.visualize:
            self.draw_radius([1,0,0])


    def add_point(self,x,y,z):
        new_contact = np.array([x,y,z])
        self.contact_points = np.vstack((self.contact_points, new_contact))
        self.c_z = np.mean(self.contact_points[:,2])
        self.plot_point(x,y,z, [1,0,0])


    def get_est(self):
        return [self.c_x, self.c_y, self.c_z, self.c_r]


    def get_error(self):
        return self.error


    def get_result(self):
        if float(self.num_inliers)/float(self.num_attempted_probes) > self.classification_error_thresh:
            return True
        else:
            return False


    def most_recent_point(self):
        p = Point()
        if len(self.contact_points) > 0:
            p.x = self.contact_points[len(self.contact_points)-1,0]
            p.y = self.contact_points[len(self.contact_points)-1,1]
            p.z = self.contact_points[len(self.contact_points)-1,2]
        return p


    def set_probe_attempts(self, attempted):
        self.num_attempted_probes = attempted


    def point_count(self):
        return len(self.contact_points)


    def print_results(self,valid_contact_points):
        print "\n-----------------------"
        print "Landmine Survey Results"
        print "-----------------------"
        if valid_contact_points:
            print "Centre X: %0.1f" % self.c_x
            print "Centre Y: %0.1f" % self.c_y
            print "Radius: %0.1f" % self.c_r
            print "-----------------------"
            print "Error: %0.3f" % self.error
            print "# Attempts:", self.num_attempted_probes
            print "# Contact:", len(self.contact_points)
            print "# Inliers:", self.num_inliers
            print "Inliers / Attempts: %0.2f%%" % (float(self.num_inliers)/float(self.num_attempted_probes)*100)
            print "Landmine?:", self.get_result()
            print "-----------------------"
            print "points:\n", self.contact_points
        else:
            print "Landmine?: False! We had no contact points"
        print "-----------------------\n"


    def reset_est(self):
        self.contact_points = np.array([], dtype=np.int64).reshape(0,3)

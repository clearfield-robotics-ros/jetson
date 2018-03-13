import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Int16, Int16MultiArray
import numpy as np
from circle_intersection import Geometry
from visualization_msgs.msg import Marker
from sklearn.cluster import KMeans
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

        self.contact_viz_id = 0
        self.contact_viz_pub = rospy.Publisher('probe_contact_viz', Marker, queue_size=10)

        self.visualize = True

        self.angle_range = rospy.get_param('third_stage_probe_angle')


    def draw_radius(self,col):

        msg = Marker()
        msg.header.frame_id = "gantry"
        # msg.header.frame_id = "probe_tip" # DEBUG
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

        # msg = Marker()
        # msg.ns = "probe_intersection_viz"
        # msg.action = msg.DELETEALL
        # self.contact_viz_pub.publish(msg)

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


    def hough(self):
        if len(self.contact_points) > 1:
            ### get Circular Intersections and plot
            intersection = np.array([], dtype=np.int64).reshape(0,2)
            for i in range(0,self.point_count()):
                for j in range(0,i):
                    p = self.g.circle_intersection( np.append(self.contact_points[i,0:2],self.radius),
                                                    np.append(self.contact_points[j,0:2],self.radius) )
                    for j in range(0,len(p)):
                        intersection = np.vstack(( intersection,np.array([p[j][0],p[j][1]]) ))

            ### Remove points (bounding box)
            intersection_BB = np.array([], dtype=np.int64).reshape(0,2)
            for i in range(0,len(intersection)):
                if ( intersection[i,0] > min(self.contact_points[:,0]) ): # and
                # intersection[i,1] > min(self.contact_points[:,1]) and
                # intersection[i,1] < max(self.contact_points[:,1]) ):
                    intersection_BB = np.vstack((intersection_BB,intersection[i,:]))

            intersection_BB = np.unique(intersection_BB, axis=0)

            if False:
                self.plot_intersections(intersection_BB)

            ### Determine center point
            if len(intersection_BB) > 1:
                km = KMeans(n_clusters=2)
                km.fit(intersection_BB)
                labels = km.labels_
                self.c_x = km.cluster_centers_[0,0]
                self.c_y = km.cluster_centers_[0,1]
            elif len(intersection_BB) == 1:
                self.c_x = intersection_BB[0,0]
                self.c_y = intersection_BB[0,1]
            else:
                self.c_x = np.mean(self.contact_points[:,0])
                self.c_y = np.mean(self.contact_points[:,1])
        else:
            self.c_x = self.contact_points[0,0] + self.radius
            self.c_y = self.contact_points[0,1]


    def get_est(self):
        return [self.c_x, self.c_y, self.c_z, self.c_r]


    def print_results(self):
        print "Landmine Survey Results"
        print "-----------------------"
        print "Centre X (mm):", self.c_x
        print "Centre Y (mm):", self.c_y
        print "Centre Z (mm):", self.c_z
        print "Radius (mm):", self.c_r


    def add_point(self,x,y,z):
        new_contact = np.array([x,y,z])
        self.contact_points = np.vstack((self.contact_points, new_contact))
        self.c_z = np.mean(self.contact_points[:,2])
        self.hough()

        if self.visualize:
            self.plot_point(x,y,z, [1,0,0])
            self.draw_radius([1,0,0])
            # self.plot_point(self.c_x, self.c_y, 0, [0,0,1])


    def most_recent_point(self):
        p = Point()
        if len(self.contact_points) > 0:
            p.x = self.contact_points[len(self.contact_points)-1,0]
            p.y = self.contact_points[len(self.contact_points)-1,1]
            p.z = self.contact_points[len(self.contact_points)-1,2]
        return p


    def get_sparsest_point(self):

        if len(self.contact_points) >= 2: # need two points to get started
            angle = []
            for i in range(0,len(self.contact_points)):
                a = math.atan2( -(self.contact_points[i,1] - self.get_est()[1]),
                    -(self.contact_points[i,0] - self.get_est()[0]) )
                angle.append(a)
            angle.sort()

            # angle_range = 80./180*math.pi # PARAM

            if min(angle) > -angle_range:
                angle.insert(0, -angle_range)

            if max(angle) < angle_range:
                angle.append(angle_range)

            # print "ANGLES:"
            # for i in range(0,len(angle)):
            #     print int(angle[i]*180/math.pi)
            #
            # for i in range(0,len(angle)):
            #     x = self.c_x - math.cos(angle[i])*self.c_r
            #     y = self.c_y - math.sin(angle[i])*self.c_r
            #     z = self.c_z
            #     self.plot_point(x,y,z-1,[1,1,0])

            dist = []
            for i in range(0,len(angle)-1):
                dist.append( abs( angle[i+1] - angle[i] ) )

            print dist

            M = max(dist)
            I = dist.index(M)

            print "Index:", I

            if I == 0 and angle[I] == -angle_range:
                probe_angle = -angle_range
                print "LOWER", probe_angle*180/math.pi

            elif I == len(angle)-2 and angle[I+1] == angle_range:
                probe_angle = angle_range
                print "UPPER", probe_angle*180/math.pi

            else:
                probe_angle = angle[I] + dist[I]/2
                print "MIDDLE", probe_angle*180/math.pi

            x = self.c_x - math.cos(probe_angle)*self.c_r
            y = self.c_y - math.sin(probe_angle)*self.c_r
            z = self.c_z

            # self.plot_point(x,y,z-2,[1,1,1])
            # print "PROBE ANGLE:", probe_angle

            return [x,y,z,probe_angle]
        else:
            return None


    def point_count(self):
        return len(self.contact_points)


    def fit_error(self):
        return 5


    def reset_est(self):
         self.contact_points = np.array([], dtype=np.int64).reshape(0,3)

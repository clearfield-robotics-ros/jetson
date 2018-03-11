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
        self.c_r = self.radius

        self.contact_viz_id = 0
        self.contact_viz_pub = rospy.Publisher('probe_contact_viz', Marker, queue_size=10)

        self.visualize = True


    def draw_radius(self,col):

        for i in range(0,20):

            x = self.radius*math.sin(360/20*i * math.pi/180) + self.c_x
            y = self.radius*math.cos(360/20*i * math.pi/180) + self.c_y
            z = 0

            msg = Marker()
            # msg.header.frame_id = "gantry"
            msg.header.frame_id = "probe_tip" # DEBUG
            msg.header.seq = i
            msg.header.stamp = rospy.Time.now()
            msg.ns = "probe_radius_viz"
            msg.id = i
            msg.type = 4  # line strip
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


    def plot_point(self,x,y,z,col):
        # Visualize probe point
        global contact_viz_id
        global contact_viz_pub
        msg = Marker()
        # msg.header.frame_id = "gantry"
        msg.header.frame_id = "probe_tip" # DEBUG
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
                if ( intersection[i,0] > min(self.contact_points[:,0]) ):# and
                # intersection[i,1] > min(self.contact_points[:,1]) and
                # intersection[i,1] < max(self.contact_points[:,1]) ):
                    intersection_BB = np.vstack((intersection_BB,intersection[i,:]))

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
            self.c_x = self.contact_points[0,0]
            self.c_y = self.contact_points[0,1]

        if self.visualize:
            self.plot_point(self.c_x, self.c_y, 0, [0,0,1])
            self.draw_radius([0,0,1])


    def get_est(self):
        return [self.c_x, self.c_y, self.c_r]


    def add_point(self,x,y,z):
        new_contact = np.array([x,y,z])
        self.contact_points = np.vstack((self.contact_points, new_contact))
        self.hough()

        if self.visualize:
            self.plot_point(x,y,z, [1,0,0])


    def point_count(self):
        return len(self.contact_points)


    def reset_est(self):
         self.contact_points = np.array([], dtype=np.int64).reshape(0,3)

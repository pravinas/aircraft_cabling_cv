#!/usr/bin/env python

# General Imports

import sys
import threading
from threading import Lock 

# ROS Imports

import rospy
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header, Float32
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

# GPR Imports

import numpy as np
from matplotlib import pyplot as plt
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import Matern 
from scipy import spatial, interpolate
import math

class DataRegressor():
    def __init__(self, array_in, cloud_in, norm_tolerance=1.1):
        self.name = "gpr_node" 
        self.array_in = array_in
        self.cloud_in = cloud_in
        self.marker_pub = None
        self.marker_pub2 = None
        self.marker_pub3 = None
        self.cloudLock = threading.Lock() 
        self.cloud = None
        self.frame_id = None
        self.length_fn = None
        self.norm_tolerance = norm_tolerance # The max ratio between norms. Should be > 1
        self.gp_x = GaussianProcessRegressor(kernel=1.0 * Matern(length_scale=0.05, length_scale_bounds=(1e-1, 10.0), nu=1.5)) 
        self.gp_y = GaussianProcessRegressor(kernel=1.0 * Matern(length_scale=0.05, length_scale_bounds=(1e-1, 10.0), nu=1.5)) 
        self.gp_z = GaussianProcessRegressor(kernel=1.0 * Matern(length_scale=0.05, length_scale_bounds=(1e-1, 10.0), nu=1.5)) 

    def updateCloud(self, cloud_msg):
        locked = self.cloudLock.acquire(False)
        if (locked):
            self.cloud = pc2.read_points(cloud_msg, skip_nans=True)
            self.cloudLock.release()

    def makeMarker(self, id_num, point, rgb=(1.0,1.0,1.0)):
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.id = id_num
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.02 
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color.r = rgb[0]
        marker.color.g = rgb[1] 
        marker.color.b = rgb[2]
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        return marker

    def fit(self, marker_array_msg):
        # Initialize setup
        marker_array = np.array(marker_array_msg.markers)
        self.frame_id = marker_array[0].header.frame_id
        X = np.array([0.1* marker.id for marker in marker_array])
        y_x = np.array([marker.pose.position.x for marker in marker_array])
        y_y = np.array([marker.pose.position.y for marker in marker_array])
        y_z = np.array([marker.pose.position.z for marker in marker_array])

        # Interpolate and refine
        if self.cloud == None:
            print "No cloud in buffer when doing processing."
            return
        self.cloudLock.acquire()
        cloud_np = np.array(list(self.cloud))[:,0:3]
        cloud_kdtree = spatial.KDTree(cloud_np)
        prev_norm = 100
        new_norm = 1
        print "begin loop"

        while prev_norm/new_norm > self.norm_tolerance or new_norm/prev_norm > self.norm_tolerance:
            # Fit GPR Models
            self.gp_x.fit(X[:, np.newaxis], y_x) 
            self.gp_y.fit(X[:, np.newaxis], y_y) 
            self.gp_z.fit(X[:, np.newaxis], y_z) 

            # Insert midpoint values into array
            midpoints = (np.delete(X, -1, -1) + np.delete(X, 0, -1))/2.0
            X = np.sort(np.append(X, midpoints))
            guesses = zip(self.gp_x.predict(X[:, np.newaxis]), self.gp_y.predict(X[:, np.newaxis]), self.gp_z.predict(X[:, np.newaxis]))

            # Update models
            distances, cloud_indices = cloud_kdtree.query(guesses, eps=0, distance_upper_bound=.2)
            refined_values = cloud_np[cloud_indices]
            y_x, y_y, y_z = refined_values.T

            prev_norm = new_norm
            new_norm =  np.linalg.norm(distances, np.inf)
        self.cloudLock.release()
        print "GPR complete" 


        # Renormalize
        # markers = MarkerArray()
        guess_lengths = np.linspace(X[0], X[-1], num=500)
        x_coords = self.gp_x.predict(guess_lengths[:, np.newaxis])
        y_coords = self.gp_y.predict(guess_lengths[:, np.newaxis])
        z_coords = self.gp_z.predict(guess_lengths[:, np.newaxis])
        points = np.vstack((x_coords, y_coords, z_coords)).T 
        prev_pt = points[0]
        lengths = []
        lastlen = 0
        for (i, point) in enumerate(points):
            added_len = math.sqrt((point[0]-prev_pt[0])**2 + (point[1]-prev_pt[1])**2 + (point[2]-prev_pt[2])**2)
            lastlen = lastlen + added_len
            prev_pt = point
            lengths.append(lastlen)

            # marker = self.makeMarker(i, point) 
            # markers.markers.append(marker)
        norm_factor = lengths[-1]/X[-1]
        lengths = lengths / norm_factor
        # self.marker_pub.publish(markers)
        # print "markers published"

        # Fit spline to normalized curve
        self.length_fn = lambda x: (np.interp(x, lengths, x_coords), np.interp(x, lengths, y_coords), np.interp(x, lengths, z_coords))

        # Output these in marker format
        # markers = MarkerArray()
        # markers.markers.append(self.makeMarker(0, self.length_fn(0.15),rgb=(0.4,0.4,0.4)))
        # markers.markers.append(self.makeMarker(1,self.length_fn(0.35),rgb=(0.5,0.5,1.0)))
        # markers.markers.append(self.makeMarker(2,self.length_fn(0.55),rgb=(0.5,1.0,0.5)))
        # markers.markers.append(self.makeMarker(3,self.length_fn(0.75),rgb=(0.2,0.5,0.7)))
        # markers.markers.append(self.makeMarker(4,self.length_fn(0.95),rgb=(0.7,0.2,0.7)))
        # self.marker_pub2.publish(markers)
        # print "markers published2"
    
    def find_location(self, float_msg):
        float_val = float_msg.data
        if self.length_fn==None:
            return
        marker = self.makeMarker(0, self.length_fn(float_val), rgb=(0.8, 0.2, 0.4))
        self.marker_pub3.publish(marker)
    
    def run(self):
        rospy.init_node(self.name, anonymous=True)
        rospy.Subscriber(self.array_in, MarkerArray, self.fit)    
        rospy.Subscriber(self.cloud_in, PointCloud2, self.updateCloud)
        rospy.Subscriber("/input_floats", Float32, self.find_location)
        # self.marker_pub = rospy.Publisher("/gpr_cable", MarkerArray, queue_size=100)
        # self.marker_pub2 = rospy.Publisher("/gpr_markers", MarkerArray, queue_size=100)
        self.marker_pub3 = rospy.Publisher("/markers_out", Marker, queue_size=100)
        rospy.spin()

if __name__ == '__main__':
    print "Usage: rosrun pcl_sandbox gpr_ros.py <MarkerArray in> <PointCloud2 cable>"
    array_in = sys.argv[1]
    cable_pc = sys.argv[2]
    DataRegressor(array_in, cable_pc).run()

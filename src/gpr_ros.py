#!/usr/bin/env python

# General Imports

import sys
import threading
from threading import Lock 

# ROS Imports

import rospy
import std_msgs.msg
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

# GPR Imports

import numpy as np
from matplotlib import pyplot as plt
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import Matern 
from scipy import spatial

class DataRegressor():
    def __init__(self, array_in, cloud_in, norm_tolerance=1.1):
        self.name = "gpr_node" 
        self.array_in = array_in
        self.cloud_in = cloud_in
        self.marker_pub = None
        self.cloudLock = threading.Lock() 
        self.cloud = None
        self.frame_id = None
        self.norm_tolerance = norm_tolerance # The max ratio between norms. Should be > 1
        self.gp_x = GaussianProcessRegressor(kernel=1.0 * Matern(length_scale=0.05, length_scale_bounds=(1e-1, 10.0), nu=1.5)) 
        self.gp_y = GaussianProcessRegressor(kernel=1.0 * Matern(length_scale=0.05, length_scale_bounds=(1e-1, 10.0), nu=1.5)) 
        self.gp_z = GaussianProcessRegressor(kernel=1.0 * Matern(length_scale=0.05, length_scale_bounds=(1e-1, 10.0), nu=1.5)) 

    def updateCloud(self, cloud_msg):
        locked = self.cloudLock.acquire(False)
        if (locked):
            self.cloud = pc2.read_points(cloud_msg, skip_nans=True)
            self.cloudLock.release()

    def makeMarker(self, id_num, point):
        marker = Marker()
        marker.header = std_msgs.msg.Header()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.frame_id
        marker.id = id_num
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.03 
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
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

        # Output these in marker format
        markers = MarkerArray()
        for i in range(len(X)):
            marker = self.makeMarker(i, (y_x[i], y_y[i], y_z[i])) 
            markers.markers.append(marker)
        self.marker_pub.publish(markers)

        # Renormalize
        points = zip(y_x, y_y, y_z)
        prev_pt = points[0]
        lengths = []
        lastlen = 0
        for (i, point) in enumerate(points):
            added_len = (point[0]-prev_pt[0])**2 + (point[1]-prev_pt[1])**2 + (point[2]-prev_pt[2])**2
            lastlen = lastlen + added_len
            prev_pt = point
            lengths.append(lastlen)
        # lengths is the renormalized lengths. what do we do now?
    
    def run(self):
        rospy.init_node(self.name, anonymous=True)
        #self.pub = rospy.Publisher(self.ptCloudTopicOut, PointCloud2, queue_size=1)
        rospy.Subscriber(self.array_in, MarkerArray, self.fit)    
        rospy.Subscriber(self.cloud_in, PointCloud2, self.updateCloud)
        self.marker_pub = rospy.Publisher("/gpr_markers", MarkerArray, queue_size=100)
        rospy.spin()

if __name__ == '__main__':
    print "Usage: rosrun pcl_sandbox gpr_ros.py <MarkerArray in> <PointCloud2 cable>"
    array_in = sys.argv[1]
    cable_pc = sys.argv[2]
    DataRegressor(array_in, cable_pc).run()

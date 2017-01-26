#!/usr/bin/env python

# General Imports

import sys
import argparse
import math
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

class DataRegressor():
    def __init__(self, array_in, cloud_in, cable_length, debug=True, norm_tolerance=1.1):
        ## Constructor for the DataRegressor Class
        # 
        # @param[in] array_in MarkerArray topic corresponding to physical markers on cable
        # @param[in] cloud_in PointCloud2 topic containing at least the whole cable
        # @param[in] cable_length Length of the cable in meters
        # @param[in] norm_tolerance Error tolerance between successive iterations of GPR.
        self.name = "gpr_node" 
        self.array_in = array_in
        self.cloud_in = cloud_in
        self.cable_length = float(cable_length)
        self.debug = debug
        self.norm_tolerance = norm_tolerance
        self.marker_pub = None

        if self.debug:
            self.cable_pub = None

        self.cloudLock = threading.Lock() 
        self.cloud = None
        self.frame_id = None
        self.length_fn = None
        self.gp_x = GaussianProcessRegressor(kernel=1.0 * Matern(length_scale=0.05, length_scale_bounds=(1e-1, 10.0), nu=1.5)) 
        self.gp_y = GaussianProcessRegressor(kernel=1.0 * Matern(length_scale=0.05, length_scale_bounds=(1e-1, 10.0), nu=1.5)) 
        self.gp_z = GaussianProcessRegressor(kernel=1.0 * Matern(length_scale=0.05, length_scale_bounds=(1e-1, 10.0), nu=1.5)) 

    def updateCloud(self, cloud_msg):
        ## Process and store cloud
        #
        # @param[in] cloud_msg PointCloud2 message.
        
        locked = self.cloudLock.acquire(False)
        if (locked):
            self.cloud = pc2.read_points(cloud_msg, skip_nans=True)
            self.cloudLock.release()

    def makeMarker(self, id_num, point, rgb=(1.0,1.0,1.0)):
        ## Make a std_msgs/Marker object
        #
        # @param id_num The id number corresponding to the marker. Note that these must be unique.
        # @param point An (x,y,z) tuple where the marker should be located in space
        # @param rgb A tuple corresponding to the RGB values on a scale from 0.0 to 1.0
        # 
        # @return A std_msgs/Marker object

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
        ## Fit function to marker array data.
        #
        # This function assumes that the cloud data is 
        # already populated, and that the markers are sorted
        # in order of their direction along the cable.
        #
        # After fitting GPR to the initial MarkerArray, we read
        # and refine the curve to find the nearest point in the 
        # PointCloud to the midpoint between two points.
        #
        # @param[in] marker_array_msg The MarkerArray that triggered this function.

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
        #cloud_np = np.array(filter(lambda x: math.isnan(x) == False, self.cloud))[:,0:3]
        cloud_np = np.array(list(self.cloud))[:,0:3]
        cloud_kdtree = spatial.KDTree(cloud_np)
        prev_norm = 100
        new_norm = 1
        print "begin loop"
        num_iterations_yet=0

        while prev_norm/new_norm > self.norm_tolerance or new_norm/prev_norm > self.norm_tolerance:
            num_iterations_yet+=1
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
        if self.debug:
            print >> sys.stderr, num_iterations_yet
        print "GPR complete" 


        # Renormalize
        if self.debug:
            markers = MarkerArray()
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

            if self.debug:
                marker = self.makeMarker(i, point) 
                markers.markers.append(marker)
        norm_factor = lengths[-1]/self.cable_length
        lengths = np.array(lengths) / norm_factor
        if self.debug:
            self.cable_pub.publish(markers)

        # Fit spline to normalized curve
        self.length_fn = lambda l: (
                np.interp(self.cable_length - l, lengths, x_coords), 
                np.interp(self.cable_length - l, lengths, y_coords), 
                np.interp(self.cable_length - l, lengths, z_coords))

        
    ## @fn DataRegressor.length_fn(l)
    # @brief Parametric function describing the shape of the cable
    # @param[in] l length along the cable to find coordinate of
    # @return (x,y,z) location of location along the cable
    
    def find_location(self, float_msg):
        ## Finds where a given value along the cable is
        #
        # @param[in] float_msg std_msgs/Float32 message containing length along cable
        # @return Marker corresponding to the location of that float
        float_val = float_msg.data
        if self.length_fn==None:
            return
        marker = self.makeMarker(0, self.length_fn(float_val), rgb=(0.8, 0.2, 0.4))
        self.marker_pub.publish(marker)
    
    def run(self):
        ## Run ROS node
        rospy.init_node(self.name, anonymous=True)
        rospy.Subscriber(self.array_in, MarkerArray, self.fit)    
        rospy.Subscriber(self.cloud_in, PointCloud2, self.updateCloud)
        rospy.Subscriber("/cv/gpr/input_floats", Float32, self.find_location)
        self.marker_pub = rospy.Publisher("/cv/gpr/marker_estimate", Marker, queue_size=100)
        if self.debug:
            self.cable_pub = rospy.Publisher("/cv/gpr/debug_markers", MarkerArray, queue_size=100)
        rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--markers-in", help="std_msgs/MarkerArray topic containing points along the cable.", default="/cv/kmeans")
    parser.add_argument("-c", "--cloud-in", help="Source sensor_msgs/PointCloud2 topic", default = "/camera/depth_registered/points")
    parser.add_argument("-l", "--length", help="Total length of the cable in meters.", type=float, default=1.0)
    parser.add_argument("-d", "--debug", help="Set flag if debug output should be shown", action="store_true")
    parser.add_argument("-t", "--tolerance", help="Ratio between successive approximations of GPR. Must be greater than 1, but not by much.", default=1.1, type=float)
    args = parser.parse_args(rospy.myargv()[1:])
    
    dataRegressor = DataRegressor(args.markers_in, args.cloud_in, args.length, debug=args.debug, norm_tolerance=args.tolerance)
    dataRegressor.run()

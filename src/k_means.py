#!/usr/bin/env python

## @file k_means.py
# @author Pravina Samaratunga

import sys
import time
import argparse
import rospy
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import KMeans as KM
import numpy

class KMeans:
    def __init__(self, ptCloudIn, markerArrayOut, num_means=10, debug=False):
        ## Constructor for the KMeans class
        # 
        # @param[in] ptCloudIn A point cloud topic with clustered data
        # @param[out] markerArrayOut A marker array topic to publish the cluster centers
        # @param[in] num_means Number of clusters in the data
        self.ptCloudTopicIn = ptCloudIn
        self.markerArrayTopicOut = markerArrayOut
        self.num_means = num_means
        self.debug = debug
        self.pub = None

    def k_means(self, msg):
        ## Perform K Means on the data
        #
        # @param msg A sensor_msgs/PointCloud2 message with num_means clusters
        starttime = time.clock()
        points = pc2.read_points(msg, field_names=["x","y","z","rgb"], skip_nans=True)
        data = []

        for point in points:
            data += [point[0:3]]

        data = numpy.array(data)
        kmeans = KM(n_clusters = self.num_means)
        kmeans.fit(data)
        print kmeans.cluster_centers_
        centers = sorted(kmeans.cluster_centers_, key=lambda p: p[0])

        markers = MarkerArray()
        for id_num, center in enumerate(centers):
            centerMarker = self.makeMarker(msg.header, id_num, center)
            markers.markers.append(centerMarker)

        self.pub.publish(markers)
        
        endtime = time.clock()
        if self.debug:
            print >> sys.stderr, "KMeans time: " + str(endtime - starttime)

    def makeMarker(self, header, id_num, point):
        ## Make a std_msgs/Marker object
        #
        # @param header A ROS Header object for the marker
        # @param id_num The id number corresponding to the marker. Note that these must be unique.
        # @param point An (x,y,z) tuple where the marker should be located in space
        # 
        # @return A std_msgs/Marker object

        marker = Marker()
        marker.header = header
        marker.id = id_num
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.3
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        return marker
       
    def run(self):
        ## Run the color filter node
        rospy.init_node("k_means", anonymous=True)
        
        self.pub = rospy.Publisher(self.markerArrayTopicOut, MarkerArray, queue_size=100)
        rospy.Subscriber(self.ptCloudTopicIn, PointCloud2, self.k_means)    
        
        rospy.spin()
        
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", help="The input sensor_msgs/PointCloud2 topic to find the means", default="/cv/filtered")
    parser.add_argument("-o", "--output", help="The output std_msgs/MarkerArray topic for the means.", default="/cv/kmeans")
    parser.add_argument("-n", "--num-means", help="Number of means to find", type=int, default=10)
    parser.add_argument("-d", "--debug", help="Print debugging output", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])
    
    kmeans = KMeans(args.input, args.output, args.num_means, args.debug)
    kmeans.run()
    

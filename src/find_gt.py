#!/usr/bin/env python

import sys
import argparse
import rospy
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from math import sqrt
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import KMeans as KM
import numpy

class KMeans:
    def __init__(self, ptCloudTopicIn, markerTopicOut, num_means=5):
        self.ptCloudTopicIn = ptCloudTopicIn
        self.markerTopicOut = markerTopicOut
        self.num_means = num_means
        self.pub = None

    def k_means(self, msg):
        points = pc2.read_points(msg, skip_nans=True)
        data = []

        for point in points:
            data += [point[0:3]]

        data = numpy.array(data)
        kmeans = KM(n_clusters = self.num_means)
        kmeans.fit(data)
        print kmeans.cluster_centers_
        counts = []
        for center in kmeans.cluster_centers_:
            count = 0
            for point in data:
                if sqrt((point[0]-center[0])**2 + (point[1]-center[1])**2 + (point[2]-center[2])**2) < 0.03:
                    count += 1
            counts.append((center,count))
        marker_point = max(counts, key=lambda x:x[1])[0]
        marker = self.makeMarker(msg.header, 0, marker_point)
        self.pub.publish(marker)

    def makeMarker(self, header, id_num, point):
        marker = Marker()
        marker.header = header
        marker.id = id_num
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        marker.color.r = 0.3
        marker.color.g = 0.2
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        return marker
       
    def run(self):
        rospy.init_node("find_gt_node", anonymous=True)
        
        self.pub = rospy.Publisher(self.markerTopicOut, Marker, queue_size=100)
        rospy.Subscriber(self.ptCloudTopicIn, PointCloud2, self.k_means)    
        
        rospy.spin()
        
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--input", help="The input sensor_msgs/PointCloud2 topic to find the means", default="/cv/filtered")
    parser.add_argument("-o", "--output", help="The output std_msgs/MarkerArray topic for the means.", default="/cv/gt_marker")
    parser.add_argument("-n", "--num-means", help="Number of means to find", type=int, default=5)
    args = parser.parse_args(rospy.myargv()[1:])
    
    kmeans = KMeans(args.input, args.output, args.num_means)
    kmeans.run()
    

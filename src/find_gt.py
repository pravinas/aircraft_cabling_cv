#!/usr/bin/env python

import sys
import rospy
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from math import sqrt
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import KMeans as KM
import numpy

# Assumes only one blob of color color
class KMeans:
    def __init__(self, name, color, ptCloudTopicIn, markerTopicOut, num_means=5):
        self.name = name
        self.color = color
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
        kmeans = KM(n_clusters = num_means)
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
        rospy.init_node(self.name, anonymous=True)
        
        self.pub = rospy.Publisher(self.markerTopicOut, Marker, queue_size=100)
        rospy.Subscriber(self.ptCloudTopicIn, PointCloud2, self.k_means)    
        
        rospy.spin()
        
        
if __name__ == '__main__':
    print "Usage: rosrun pcl_sandbox k_means.py <nodename> <color (red/blue)> <cloud in> <marker out> <num means>"
    name = sys.argv[1]
    color = sys.argv[2]
    cloud_topic = sys.argv[3]
    marker_out = sys.argv[4]
    num_means = int(sys.argv[5])
    
    kmeans = KMeans(name, color, cloud_topic, marker_out, num_means)
    
    kmeans.run()
    

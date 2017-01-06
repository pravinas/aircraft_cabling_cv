#!/usr/bin/env python

import sys
import rospy
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray
from sklearn.cluster import KMeans as KM
import numpy

# Assumes only one blob of color color
class KMeans:
    def __init__(self, name, color, ptCloudTopicIn, ptCloudTopicOut):
        self.name = name
        self.color = color
        self.ptCloudTopicIn = ptCloudTopicIn
        self.ptCloudTopicOut = ptCloudTopicOut
        self.pub = None

    def k_means(self, msg):
        points = pc2.read_points(msg, skip_nans=True)
        data = []

        for point in points:
            data += [point[0:3]]

        data = numpy.array(data)
        kmeans = KM(n_clusters = 10)
        kmeans.fit(data)
        print kmeans.cluster_centers_
        centers = sorted(kmeans.cluster_centers_, key=lambda p: p[0])

        markers = MarkerArray()
        for id_num, center in enumerate(centers):
            centerMarker = self.makeMarker(msg.header, id_num, center)
            markers.markers.append(centerMarker)

        self.pub.publish(markers)

    def makeMarker(self, header, id_num, point):
        marker = Marker()
        marker.header = header
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
       
    def run(self):
        rospy.init_node(self.name, anonymous=True)
        
        self.pub = rospy.Publisher(self.ptCloudTopicOut, MarkerArray, queue_size=100)
        rospy.Subscriber(self.ptCloudTopicIn, PointCloud2, self.k_means)    
        
        rospy.spin()
        
        
if __name__ == '__main__':
    name = sys.argv[1]
    color = sys.argv[2]
    cloud_topic = sys.argv[3]
    marker_out = sys.argv[4]
    
    kmeans = KMeans(name, color, cloud_topic, marker_out)
    
    kmeans.run()
    

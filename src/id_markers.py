#!/usr/bin/env python

import sys
import rospy
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker

# Assumes only one blob of color color
class LocateBlob:
    def __init__(self, name, color, ptCloudTopicIn, ptCloudTopicOut):
        self.name = name
        self.color = color
        self.ptCloudTopicIn = ptCloudTopicIn
        self.ptCloudTopicOut = ptCloudTopicOut
        self.pub = None
        

    def locate(self, msg):
        points = pc2.read_points(msg, field_names=["x","y","z","rgb"], skip_nans=True)
        points_out = []

        xField = PointField()
        xField.name = 'x'
        xField.offset = 0
        xField.datatype = 7
        xField.count = 1

        yField = PointField()
        yField.name = 'y'
        yField.offset = 4
        yField.datatype = 7
        yField.count = 1

        zField = PointField()
        zField.name = 'z'
        zField.offset = 8
        zField.datatype = 7
        zField.count = 1

        rgbField = PointField()
        rgbField.name = 'rgb'
        rgbField.offset = 16
        rgbField.datatype = 7
        rgbField.count = 1

        N = 0
        
        for point in points:
            x = point[0]
            y = point[1]
            z = point[2]
            rgb = point[3]
            # cast float32 to int so that bitwise operations are possible
            s = struct.pack('>f' , rgb)
            i = struct.unpack('>l', s)[0]
            
            # you can get back the float value by the inverse operations
            pack = ctypes.c_uint32(i).value
            r = (pack & 0x00FF0000) >> 16
            g = (pack & 0x0000FF00) >> 8
            b = (pack & 0x000000FF)
            if self.color == "red":
                if r > 2.0 * g and r > 1.7 * b and r > 100:
                    points_out += [[x,y,z,rgb]]
                    N += 1
            
            if self.color == "blue":
                if b > 1.7 * g and b > 1.8 * r and b > 100:
                    points_out += [[x,y,z,rgb]]
                    N += 1
          
        if N != 0:  
            print str(N) + " points in color-filtered point cloud"
            pointCloudMsg = pc2.create_cloud(msg.header, [xField, yField, zField, rgbField], points_out) 
            self.pub.publish(pointCloudMsg)
        else:
            print "no points"
       
    def run(self):
        rospy.init_node(self.name, anonymous=True)
        
        self.pub = rospy.Publisher(self.ptCloudTopicOut, PointCloud2, queue_size=1)
        rospy.Subscriber(self.ptCloudTopicIn, PointCloud2, self.locate)    
        
        rospy.spin()
        
        
if __name__ == '__main__':
    print "Usage: rosrun pcl_sandbox id_markers.py <nodename> <color (red/blue)> <PointCloud2 topic in> <PointCloud2 topic out>"
    name = sys.argv[1]
    color = sys.argv[2]
    cloud_topic = sys.argv[3]
    cloud_topic_out = sys.argv[4]
    
    locateBlob = LocateBlob(name, color, cloud_topic, cloud_topic_out)
    
    locateBlob.run()
    

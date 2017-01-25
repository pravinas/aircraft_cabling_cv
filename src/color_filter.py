#!/usr/bin/env python

import sys
import argparse
import rospy
import sensor_msgs.point_cloud2 as pc2
import ctypes
import struct
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker

# Assumes only one blob of color color
class ColorFilter:
    def __init__(self, color, ptCloudTopicIn, ptCloudTopicOut):
        self.color = color
        self.ptCloudTopicIn = ptCloudTopicIn
        self.ptCloudTopicOut = ptCloudTopicOut
        self.pub = None
        

    def filter(self, msg):
        points = pc2.read_points(msg, field_names=["x","y","z","rgb"], skip_nans=False)
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
                if (r>3.0*g and r>2.0*b and r>100) or (r>1.8*g and r>1.8*b and r>220):
                    points_out += [[x,y,z,rgb]]
                    N += 1
            
            if self.color == "blue":
                if (b>1.5*g and b>2.5*r and b>100) or (b>1.2*g and b>1.2*r and b>200):
                    points_out += [[x,y,z,rgb]]
                    N += 1
          
        if N != 0:  
            print str(N) + " points in color-filtered point cloud"
            pointCloudMsg = pc2.create_cloud(msg.header, [xField, yField, zField, rgbField], points_out) 
            self.pub.publish(pointCloudMsg)
        else:
            print "no points"
       
    def run(self):
        rospy.init_node("color_filter", anonymous=True)
        
        self.pub = rospy.Publisher(self.ptCloudTopicOut, PointCloud2, queue_size=1)
        rospy.Subscriber(self.ptCloudTopicIn, PointCloud2, self.filter)    
        
        rospy.spin()
        
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-c", "--color", choices=["red","blue"], help="Color to filter (red/blue)", required=True)
    parser.add_argument("-i", "--input", help="Input sensor_msgs/PointCloud2 topic", default="/camera/depth_registered/points")
    parser.add_argument("-o", "--output", help="Filtered point cloud output topic", default="/cv/filtered/")
    args = parser.parse_args(rospy.myargv()[1:])
    
    colorFilter = ColorFilter(args.color, args.input, args.output)
    colorFilter.run()
    

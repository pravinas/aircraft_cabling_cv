#!/usr/bin/env python

import sys
import rospy
from visualization_msgs.msg import Marker

# Assumes only one blob of color color
class CalculateDistance:
    def __init__(self, name, et, gtt):
        self.name = name
        self.estimation_marker = None
        self.ground_truth_marker = None
        self.estimation_topic = et
        self.ground_truth_topic = gtt
        self.pub = None
        
       
    def run(self):
        rospy.init_node(self.name, anonymous=True)
        
        self.pub = rospy.Publisher(self.ptCloudTopicOut, PointCloud2, queue_size=1)
        rospy.Subscriber(self.estimation_topic, Marker, lambda msg: self.estimation_marker=msg)
        rospy.Subscriber(self.ground_truth_topic, Marker, lambda msg: self.ground_truth_marker=msg)
        
        rospy.spin()
        
        
if __name__ == '__main__':
    print "Usage: rosrun pcl_sandbox id_markers.py <nodename> <color (red/blue)> <PointCloud2 topic in> <PointCloud2 topic out>"
    name = sys.argv[1]
    color = sys.argv[2]
    et = sys.argv[3]
    gtt = sys.argv[4]
    
    locateBlob = LocateBlob(name, color, et, gtt)
    
    locateBlob.run()
    

#!/usr/bin/env python

import sys
import rospy
from math import sqrt
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32

# Assumes only one blob of color color
class CalculateDistance:
    def __init__(self, name, et, gtt, trigger):
        self.name = name
        self.estimation_marker = None
        self.ground_truth_marker = None
        self.estimation_topic = et
        self.ground_truth_topic = gtt
        self.trigger_topic = trigger
        self.pub = None
        
    def calculate_vals(self, msg):
        gt = self.ground_truth_marker.pose.position
        e = self.estimation_marker.pose.position
        float_msg = Float32()
        float_msg.data = sqrt((gt.x-e.x)**2 + (gt.y-e.y)**2 + (gt.z-e.z)**2)
        self.pub(float_msg)
        
       
    def run(self):
        rospy.init_node(self.name, anonymous=True)
        
        self.pub = rospy.Publisher("/distances", Float32, queue_size=1)
        rospy.Subscriber(self.estimation_topic, Marker, lambda msg: self.estimation_marker=msg)
        rospy.Subscriber(self.ground_truth_topic, Marker, lambda msg: self.ground_truth_marker=msg)
        rospy.Subscriber(self.trigger_topic, Float32, self.calculate_vals)
        
        rospy.spin()
        
        
if __name__ == '__main__':
    name = sys.argv[1]
    et = sys.argv[2]
    gtt = sys.argv[3]
    
    locateBlob = LocateBlob(name, et, gtt)
    
    locateBlob.run()
    

#!/usr/bin/env python

import sys
import argparse
import rospy
from math import sqrt
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32

class CalculateDistance:
    def __init__(self, et, gtt, trigger):
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
        self.pub.publish(float_msg)

    def set_em(self, msg):
        self.estimation_marker=msg

    def set_gm(self, msg):
        self.ground_truth_marker=msg

    def run(self):
        rospy.init_node("distances_node", anonymous=True)
        
        self.pub = rospy.Publisher("/distances", Float32, queue_size=1)
        rospy.Subscriber(self.estimation_topic, Marker, self.set_em)
        rospy.Subscriber(self.ground_truth_topic, Marker, self.set_gm)
        rospy.Subscriber(self.trigger_topic, Float32, self.calculate_vals)
        
        rospy.spin()
        
        
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-e", "--estimation-topic", help="Marker topic containing estimate")
    parser.add_argument("-g", "--ground-truth-topic", help="Marker topic containing ground truth data.")
    parser.add_argument("-t", "--trigger", help="Float topic that will activate distance calculation when any value is passed.")
    args = parser.parse_args(rospy.myargv()[1:])
    
    locateBlob = CalculateDistance(args.estimation_topic, args.ground_truth_topic, args.trigger)
    
    locateBlob.run()
    

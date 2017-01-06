#!/usr/bin/env python

# ROS Imports

import sys
import rospy
from visualization_msgs.msg import Marker, MarkerArray

# GPR Imports

import numpy as np

from matplotlib import pyplot as plt

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import (RBF, Matern, RationalQuadratic,
                                              ExpSineSquared, DotProduct,
                                              ConstantKernel)

def gpr_matern_plot():
    kernel = 1.0 * Matern(length_scale=0.15, length_scale_bounds=(1e-1, 10.0), nu=1.5)
    fig_index = 0

    # Specify Gaussian Process
    gp = GaussianProcessRegressor(kernel=kernel)

    # Plot prior
    plt.figure(fig_index, figsize=(8, 8))
    plt.subplot(2, 1, 1)
    X_ = np.linspace(0, 5, 100)
    y_mean, y_std = gp.predict(X_[:, np.newaxis], return_std=True)
    plt.plot(X_, y_mean, 'k', lw=3, zorder=9)
    plt.fill_between(X_, y_mean - y_std, y_mean + y_std,
                        alpha=0.5, color='k')
    y_samples = gp.sample_y(X_[:, np.newaxis], 10)
    plt.plot(X_, y_samples, lw=1)
    plt.xlim(0, 5)
    plt.ylim(-3, 3)
    plt.title("Prior (kernel:  %s)" % kernel, fontsize=12)

    # Generate data and fit GP
    rng = np.random.RandomState(6)
    X = rng.uniform(0, 6, 10)[:, np.newaxis]
    y = np.sin((X[:, 0] - 2.5) ** 2)
    gp.fit(X, y)

    # Plot posterior
    plt.subplot(2, 1, 2)
    X_ = np.linspace(0, 5, 100)
    y_mean, y_std = gp.predict(X_[:, np.newaxis], return_std=True)
    plt.plot(X_, y_mean, 'k', lw=3, zorder=9)
    plt.fill_between(X_, y_mean - y_std, y_mean + y_std,
                        alpha=0.5, color='k')

    y_samples = gp.sample_y(X_[:, np.newaxis], 10)
    plt.plot(X_, y_samples, lw=1)
    plt.scatter(X[:, 0], y, c='r', s=50, zorder=10)
    plt.xlim(0, 5)
    plt.ylim(-3, 3)
    plt.title("Posterior (kernel: %s)\n Log-Likelihood: %.3f"
                % (gp.kernel_, gp.log_marginal_likelihood(gp.kernel_.theta)),
                fontsize=12)
    plt.tight_layout()
    plt.show()

class DataRegressor():
    def __init__(self, topic_in):
        self.name = "gpr_node" 
        self.topic_in = topic_in 
        self.gp_x = GaussianProcessRegressor(kernel=1.0 * Matern(length_scale=0.15, length_scale_bounds=(1e-1, 10.0), nu=1.5)) 
        self.gp_y = GaussianProcessRegressor(kernel=1.0 * Matern(length_scale=0.15, length_scale_bounds=(1e-1, 10.0), nu=1.5)) 
        self.gp_z = GaussianProcessRegressor(kernel=1.0 * Matern(length_scale=0.15, length_scale_bounds=(1e-1, 10.0), nu=1.5)) 

    def fit(self, marker_array_msg):
        X = []
        y_x = []
        y_y = []
        y_z = []
        for marker in marker_array_msg.markers: #seed the GPR process
            X.append(10 * marker.id)
            point = marker.pose.position
            y_x.append(point[0])
            y_y.append(point[1])
            y_z.append(point[2])
        self.gp_x.fit(X, y_x) 
        self.gp_y.fit(X, y_y) 
        self.gp_z.fit(X, y_z) 
        print "~~~"
        print X
        print "~"
        print y_x
        print "~"
        print self.gp_x.predict(15)

    
    def run(self):
        rospy.init_node(self.name, anonymous=True)
        #self.pub = rospy.Publisher(self.ptCloudTopicOut, PointCloud2, queue_size=1)
        rospy.Subscriber(self.topic_in, MarkerArray, self.fit)    
        rospy.spin()

if __name__ == '__main__':
    print "Usage: rosrun pcl_sandbox gpr_ros.py <MarkerArray in>"
    array_in = sys.argv[1]
    DataRegressor(array_in)

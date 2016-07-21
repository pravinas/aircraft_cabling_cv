TODO: Document how to take pics with a clean install of Ubuntu 14.04 + USB 2.0 ports and an Xtion

# How to use ASUS Xtion with ROS Indigo

## Prerequisites

This documentation assumes that you are running Ubuntu 14.04.

## Installation

If you do not have ROS Indigo installed, follow the instructions on [this tutorial](http://wiki.ros.org/indigo/Installation/Ubuntu).

Once you have done this, install the OpenNI drivers and launch files.

```
sudo apt-get install ros-indigo-openni2-camera
sudo apt-get install ros-indigo-openni2-launch
```

## Usage

Plug in the ASUS Xtion into a USB port on your computer. Run 

TODO: Make sure this command is actually right.
```
roslaunch openni2-launch openni2.launch
```

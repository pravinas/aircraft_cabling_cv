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

Before doing anything else, run `roscore` in a terminal window, and plug in the ASUS Xtion into a USB port.

In order to begin collecting the data, run the following line in a new terminal:

```
roslaunch openni2_launch openni2.launch
```

In another terminal, run the following line to collect the data that is being output:

```
rosbag record -o data /camera/rgb/image_raw /camera/depth_registered/points /tf /camera/rgb/camera_info
```

Press ctrl+C after a few seconds to stop recording. Since the bag files can get quite large quite quickly, the recording should not be too long. You should now have a file called `data-<timestamp>.bag` in the current working directory. This file contains all the data necessary to visualize information collected by the Xtion.

## Troubleshooting

### Error Message:

```
[openni2.launch] is neither a launch file in package [openni2_launch] nor is [openni2_launch] a launch file name
The traceback for the exception was written to the log file
```

Close all open terminals. Open a new terminal, and run `roscore`. In another new terminal, run the following lines:

```
source /opt/ros/indigo/setup.bash
roslaunch openni2_launch openni2.launch
```

In yet another terminal, collect the bag file as above, with:

```
rosbag record -o data /camera/rgb/image_raw /camera/depth_registered/points /tf /camera/rgb/camera_info
```

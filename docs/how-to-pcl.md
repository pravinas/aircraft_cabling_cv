# How to use The Point Cloud Library (PCL) with ROS Indigo.

## Prerequisites

This documentation assumes that you are running Ubuntu 14.04.

## Installation

If you do not have ROS Indigo installed, follow the instructions on [this tutorial](http://wiki.ros.org/indigo/Installation/Ubuntu).

Once you have done this, install the Point Cloud Library with:

```
sudo apt-get install libpcl-all
```

## Utilizing PCL in your Project

Now that PCL is installed on your computer, you can start working with its powerful array of libraries. Enter your workspace and create a catkin package, as per [this tutorial](http://wiki.ros.org/catkin/Tutorials/CreatingPackage), with the packages `pcl_conversions`, `pcl_ros`, and `roscpp` as dependencies.

You will also need to add the following lines to your `package.xml` file.

```
<build_depend>libpcl-all-dev</build_depend>
<run_depend>libpcl-all-dev</run_depend>
```

Furthermore, verify that the following lines are present:

```
  <build_depend>pcl_conversions</build_depend>
  <build_depend>pcl_ros</build_depend>
  <build_depend>roscpp</build_depend>
  <run_depend>pcl_conversions</run_depend>
  <run_depend>pcl_ros</run_depend>
  <run_depend>roscpp</run_depend>
```

Make sure that your `CMakeLists.txt` includes the following basic PCL packages. Search for the line containing `find_package(catkin REQUIRED COMPONENTS` and add the following lines between that line and the close parenthesis if they are not already there.

```
pcl_conversions
pcl_ros
roscpp
```

You should now be able to run any C++ files that utilize PCL's libraries in this package.

## Further resources

[ROS Wiki PCL Tutorial for catkin](wiki.ros.org/pcl/Tutorial)

[PCL Official Tutorials](www.pointclouds.org/documentation/tutorials): Warning, many of these tutorials utilize rosbuild, rather than catkin, so you need to be aware of the difference.


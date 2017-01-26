# List and explanation of topics used in data_collection.launch

## Input topics

> /camera/depth_registered/points

\[sensor_msgs/PointCloud2\] This topic is the default RGB Point Cloud from the ASUS Xtion Pro.

> /cv/gpr/input_floats

\[std_msgs/Float32\] A stream of floats to be processed by the GP program.

> /trigger

\[std_msgs/Float32\] Trigger for distance between estimated and ground truth positions to be calculated.

## Output topics

> /cv/gpr/cable_reconstruction

\[visualization_msgs/MarkerArray\] Points on the estimated location of the cable.

> /cv/gpr/marker_estimate

\[visualization_msgs/Marker\] A point corresponding to the location a distance *d* along the cable in 3-space, where *d* is given by the most recent message along /cv/gpr/input_floats

> /distances

\[std_msgs/Float32\] Distance between ground truth marker and estimated location along the cable.

## Internal topics

> /cv/filtered/blue

\[sensor_msgs/PointCloud2\] A point cloud consisting entirely of the blue points in the original image.

> /cv/filtered/red

\[sensor_msgs/PointCloud2\] A point cloud consisting entirely of the red points in the original image.

> /cv/find_gt/gt_marker

\[visualization_msgs/Marker\] The location of the blue ground truth marker.

> /cv/kmeans

\[visualization_msgs/MarkerArray\] The locations of the red markers along the cable.


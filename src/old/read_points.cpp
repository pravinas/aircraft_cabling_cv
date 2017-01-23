#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

#define POINT_CLOUD_MSG_QUEUE_LENGTH 3

// Global Vars

ros::Publisher pub;

// Function triggered every time a message is published to the relevant topic.
void readerCallback (const sensor_msgs::PointCloud2ConstPtr& input)
{
  const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*input, *cloud);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (cloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
  {
    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    return;
  }
  
  std::cerr << "\n\n" << std::endl;

  std::cerr << "Total Points: " << cloud->points.size() << " " << std::endl;

  std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "point_cloud_reader");
    ros::NodeHandle nh;

    if (argc != 2)
    {
        printf("Usage: rosrun pcl_sandbox pcl_sandbox <topic_name>\n");
        return 1;
    }

    ros::Subscriber sub = nh.subscribe(argv[1], POINT_CLOUD_MSG_QUEUE_LENGTH, readerCallback); 
    pub = nh.advertise<sensor_msgs::PointCloud2>("processed_cloud", POINT_CLOUD_MSG_QUEUE_LENGTH);
    ros::spin();

    return 0;
}

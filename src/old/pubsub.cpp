#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing_rgb.h>

#define COLOR_RED 0xff0000
#define COLOR_BLUE 0x0000ff

typedef pcl::PointXYZRGB PointT;

ros::Publisher pc_pub;

void filterCloud(pcl::PointCloud<PointT>::Ptr input,
        pcl::PointCloud<PointT>::Ptr output, 
        pcl::PointCloud<pcl::Normal>::Ptr output_normals)
{
  // All the objects needed
  pcl::PassThrough<PointT> pass;
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

  // Build a passthrough filter to remove spurious NaNs
  pass.setInputCloud (input);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.5);
  pass.filter (*output);

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (output);
  ne.setKSearch (50);
  ne.compute (*output_normals);
}

void removeModel(pcl::PointCloud<PointT>::Ptr cloud, 
        pcl::PointCloud<pcl::Normal>::Ptr normals, 
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg,
        bool negative)
{
  // Objects needed
  pcl::ExtractIndices<PointT> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;

  //Data
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setInputCloud (cloud);
  seg.setInputNormals (normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers, *coefficients);

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud);
  extract.setIndices (inliers);

  // Remove the planar inliers, extract the rest
  extract.setNegative (negative);
  extract.filter (*cloud);
  extract_normals.setNegative (negative);
  extract_normals.setInputCloud (normals);
  extract_normals.setIndices (inliers);
  extract_normals.filter (*normals);
}


void readerCallback (const sensor_msgs::PointCloud2ConstPtr& input)
{
  const pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
  const pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  pcl::fromROSMsg(*input, *cloud);
  std::cerr << "PointCloud has: " << cloud->points.size () << " data points." << std::endl;

  pcl::RegionGrowingRGB<PointT> reg_seg;

  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> sac_seg; 
  sac_seg.setOptimizeCoefficients (true);
  sac_seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  sac_seg.setNormalDistanceWeight (0.1);
  sac_seg.setMethodType (pcl::SAC_RANSAC);
  sac_seg.setMaxIterations (200);
  sac_seg.setDistanceThreshold (0.03);

  filterCloud(cloud, cloud, normals);
  removeModel(cloud, normals, sac_seg, true);
  sac_seg.setDistanceThreshold (0.01);
  removeModel(cloud, normals, sac_seg, true);

  //sac_seg.setModelType (pcl::SACMODEL_LINE);
  //sac_seg.setDistanceThreshold (0.15);
  //removeModel(cloud, normals, sac_seg, false); 
  
  std::cerr << "PointCloud representing the extracted component: " << cloud->points.size () << " data points." << std::endl;

  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  pc_pub.publish(output);
}

int main(int argc, char** argv)
{
    if (argc < 2){
        std::cerr << "Usage: rosrun pcl_sandbox pcl_sandbox <PointCloud2 topic>" <<std::endl;
        return -1;
    }

    ros::init(argc, argv, "pcl_sandbox");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe(argv[1], 2, readerCallback); 
    pc_pub = n.advertise<sensor_msgs::PointCloud2>("extracted_cable", 2); 
    ros::spin();
    return 0;
}

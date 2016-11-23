#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/filters/conditional_removal.h>
/*#include "linear_comparison.hpp"*/

ros::Publisher pub;

pcl::ConditionBase<pcl::PointXYZRGB> red_condition (void)
    /**
     * Given a point, this PCL Condition will evaluate True if a point is red,
     * and false otherwise.
     */
{
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr red_cond (new pcl::ConditionAnd<pcl::PointXYZRGB ());
    // r > 1.1 * g
    //red_cond->addComparison();// TODO: Implement my own class for this
    // r > 1.1 * b
    return NULL;
}

void find_markers (const sensor_msgs::PointCloud2ConstPtr& input)
    /**
     * Given PointCloud Data, find markers of a given color range. 
     * TODO: Specifically search for red markers.
     */
{


    
}

int main(int argc, char** argv)
{
    if (argc < 3){
        std::cerr << "Usage: id_markers /input_pointcloud /output_pointcloud" << std::endl;
        return 1;
    }
    ros::init(argc, argv, "id_markers");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe(argv[1], 2, find_markers); 
    pub = n.advertise<sensor_msgs::PointCloud2>(argv[2], 2); 
    ros::spin();
    return 0;
}

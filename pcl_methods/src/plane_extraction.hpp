#include <iostream>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

struct Colour {
    Colour(int rr, int gg, int bb) : r(rr), g(gg), b(bb){};
    int r;
    int g;
    int b;
};

void pcl_callback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& msg);
int extractPlanesByProportion(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
    float proportion,
    float tolerance = 0.05,
    Colour colour = Colour(-1,-1,-1));

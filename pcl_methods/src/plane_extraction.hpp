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
#include <pcl/common/common.h>
#include <pcl/cloud_iterator.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

struct Colour {
    Colour(int rr, int gg, int bb) : r(rr), g(gg), b(bb){};
    int r;
    int g;
    int b;
};

template <typename T>
struct CloudBounds {
    CloudBounds(T l, T u) : lower(l), upper(u) {}
    T lower;
    T upper;
};

void printBounds(CloudBounds<pcl::PointXYZRGB> bounds){
    std::cout << bounds.lower.x << " <= x <= " << bounds.upper.x << std::endl;
    std::cout << bounds.lower.y << " <= y <= " << bounds.upper.y << std::endl;
    std::cout << bounds.lower.z << " <= z <= " << bounds.upper.z << std::endl;
}

pcl::PointCloud<pcl::PointXYZRGB> boundBoxPointCloud(CloudBounds<pcl::PointXYZRGB> bounds);

void pcl_callback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& msg);

// std::vector<pcl::PointCloud<pcl::PointXYZRGB> > extractPlanesByProportion(
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
//     float proportion,
//     float tolerance = 0.05,
//     Colour colour = Colour(-1,-1,-1));
pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractDominantPlane(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    pcl::ModelCoefficients::Ptr coefficients,
    float tolerance);

template <typename T>
CloudBounds<T> getCloudBounds(const typename pcl::PointCloud<T>::ConstPtr& cloud);
bool pointInBounds(pcl::PointXYZRGB point, CloudBounds<pcl::PointXYZRGB> bounds);
pcl::PointCloud<pcl::PointXYZRGB> getPointsInBounds(
    CloudBounds<pcl::PointXYZRGB> bounds,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const pcl::ModelCoefficients::Ptr& coefficients);


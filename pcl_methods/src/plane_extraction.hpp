#include <iostream>
#include <ros/ros.h>
#include <pclutil/pclutil.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/plane_extracted.h>
#include <string>

void pcl_callback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& msg);
void extractDominantPlane(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_org,
    pcl::ModelCoefficients::Ptr coefficients, float tolerance);

pcl::PointCloud<pcl::PointXYZRGB> getPointsInBounds(
    PCLUtil::CloudBounds<pcl::PointXYZRGB> bounds,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const pcl::ModelCoefficients::Ptr& coefficients);

pcl::PointCloud<pcl::PointXYZRGB> boundBoxPointCloud(PCLUtil::CloudBounds<pcl::PointXYZRGB> bounds);

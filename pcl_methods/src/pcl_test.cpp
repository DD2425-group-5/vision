#include <iostream>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

ros::Publisher explane;

void pcl_callback(const sensor_msgs::PointCloud2& msg){
    // convert to PCL implementation of the cloud, assume xyzrgb.
    // The following seems to work, but not sure how expensive it is
    // Does the assignment to constptr do a copy or something?
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(msg, *cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cld = cloud;
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);

    seg.setInputCloud(cld);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
    	PCL_ERROR("Could not estimate a planar model for the given dataset.");
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
    	      << coefficients->values[1] << " "
    	      << coefficients->values[2] << " " 
    	      << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
    for (size_t i = 0; i < inliers->indices.size(); ++i) {
	cloud->points[inliers->indices[i]].r = 255;
	cloud->points[inliers->indices[i]].g = 0;
	cloud->points[inliers->indices[i]].b = 0;
    }
    sensor_msgs::PointCloud2 out;
    
    pcl::toROSMsg(*cloud, out);
    explane.publish(out);
}

int main (int argc, char* argv[]) {
    ros::init(argc, argv, "pcl_test");
    ros::NodeHandle handle;
    
    ros::Subscriber depth_cloud = handle.subscribe("/camera/depth_registered/points", 1, pcl_callback);
    explane = handle.advertise<sensor_msgs::PointCloud2>("/pcl_test/extract", 10);
    
    ros::Rate loop_rate(10);
    while (ros::ok()){
	ros::spinOnce();
	loop_rate.sleep();
    }
}

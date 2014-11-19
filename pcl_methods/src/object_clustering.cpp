#include <iostream>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <iostream>

ros::Publisher cluster_pub;

void cluster_callback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& msg) {
    std::cout << "PointCloud before filtering has: " << msg->points.size() << " data points." << std::endl;
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
    vg.setInputCloud(msg);
    vg.setLeafSize(0.01f, 0.01f, 0.01f);
    vg.filter(*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size()  << " data points." << std::endl; //*

    // // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);

    int i=0, nr_points = (int)cloud_filtered->points.size();
    while (cloud_filtered->points.size() > 0.05 * nr_points)
    {
    	// Segment the largest planar component from the remaining cloud
    	seg.setInputCloud(cloud_filtered);
    	seg.segment(*inliers, *coefficients);
    	if (inliers->indices.size () == 0)
    	{
    	    std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    	    break;
    	}

    	// Extract the planar inliers from the input cloud
    	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    	extract.setInputCloud(cloud_filtered);
    	extract.setIndices(inliers);
    	extract.setNegative(false);

    	// Get the points associated with the planar surface
    	extract.filter(*cloud_plane);
    	std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

    	// Remove the planar inliers, extract the rest
    	extract.setNegative(true);
    	extract.filter(*cloud_f);
    	*cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered);

    cluster_pub.publish(cloud_filtered);

    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    // ec.setClusterTolerance(0.02); // 2cm
    // ec.setMinClusterSize(100);
    // ec.setMaxClusterSize(25000);
    // ec.setSearchMethod(tree);
    // ec.setInputCloud(cloud_filtered);
    // ec.extract(cluster_indices);

    // pcl::PCDWriter writer;
    // int j = 0;
    // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    // {
    // 	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    // 	for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
    // 	    cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    // 	cloud_cluster->width = cloud_cluster->points.size();
    // 	cloud_cluster->height = 1;
    // 	cloud_cluster->is_dense = true;

    // 	std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
    // 	std::stringstream ss;
    // 	ss << "cloud_cluster_" << j << ".pcd";
    // 	writer.write<pcl::PointXYZRGB>(ss.str(), *cloud_cluster, false); //*
    // 	j++;
    // }
}

int main (int argc, char* argv[]) {
    ros::init(argc, argv, "object_clustering");
    ros::NodeHandle handle;
    
    ros::Subscriber depth_cloud = handle.subscribe("/camera/depth_registered/points", 1, cluster_callback);
    // publish a ROS type - can automatically convert from the PCL type
    cluster_pub = handle.advertise<sensor_msgs::PointCloud2>("/object_clustering/extracted_objects", 10);
    
    ros::Rate loop_rate(10);
    while (ros::ok()){
	ros::spinOnce();
	loop_rate.sleep();
    }
}

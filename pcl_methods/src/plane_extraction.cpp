#include "plane_extraction.hpp"

ros::Publisher explane;



/**
 * Subscribing to a PCL type will automatically convert the ROS message type.
 * Need to use a Ptr here, because the setInputCloud requires a ConstPtr. If you
 * use a ConstPtr in the callback, however, you can't modify the contents of the
 * cloud and so need to do some conversions to actually allow modification of
 * the contents.
 */
void pcl_callback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& msg){
    extractPlanesByProportion(msg, 0.3, 0.08);//, Colour(255, 0, 0));
    explane.publish(msg);
}

/**
 * Extract planes which take up >= the given proportion of points in the
 * original cloud. Does multiple passes on the cloud until all planes are
 * removed or labelled. There is no upper limit on the size of the plane
 * that can be extracted.
 * !!This function modifies the object that it is given!!
 *
 * If a colour is provided as an argument, then RGB values for the points in
 * planes are set to that value. Otherwise, the points are removed from the
 * plane. A colour of (-1, -1, -1) will also result in point erasure.
 */
int extractPlanesByProportion(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
			       float proportion,
			       float tolerance,
			       Colour colour) {
    // Set a boolean if we are to erase 
    bool erase;
    if (colour.r == -1 && colour.b == -1 && colour.g == -1){
	erase = true;
    }
    
    int nExtractedPlanes = 0;
    int minPointsInPlane = (int) proportion * cloud->points.size();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(tolerance);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    int nPointsInPlane = inliers->indices.size();
    std::cerr << "Model #inliers: " << nPointsInPlane << std::endl;
    if (nPointsInPlane == 0) {
	// Couldn't find a plane, so return
    	PCL_ERROR("Could not estimate a planar model for the given dataset.");
	return nExtractedPlanes;
    } else if (nPointsInPlane < minPointsInPlane) {
	// found a plane, but smaller than minimum proportion, so no point
	// continuing.
	PCL_INFO("Extracted plane is smaller than minimum proportion.");
	return nExtractedPlanes;
    }
    std::cout << "Modifying cloud" << std::endl;

    if (erase) {
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*plane);
	*cloud = *plane; // using the swap function doesn't work for some reason
    } else { // Colour all the inliers with the requested colour
	PCL_INFO("Colouring points");
	for (size_t i = 0; i < inliers->indices.size(); ++i) {
	    cloud->points[inliers->indices[i]].r = colour.r;
	    cloud->points[inliers->indices[i]].g = colour.g;
	    cloud->points[inliers->indices[i]].b = colour.b;
	}
    }
    return nExtractedPlanes;
}

int main (int argc, char* argv[]) {
    ros::init(argc, argv, "pcl_test");
    ros::NodeHandle handle;
    
    ros::Subscriber depth_cloud = handle.subscribe("/camera/depth_registered/points", 1, pcl_callback);
    // publish a ROS type - can automatically convert from the PCL type
    explane = handle.advertise<sensor_msgs::PointCloud2>("/pcl_test/extract", 10);
    
    ros::Rate loop_rate(10);
    while (ros::ok()){
	ros::spinOnce();
	loop_rate.sleep();
    }
}

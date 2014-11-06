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
    extractDominantPlane(msg, 0.01);
    explane.publish(msg);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractDominantPlane(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    float tolerance) {
    // Stores the coefficients of the plane, defined as
    // ax + by + cz + d = 0
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>());
    // The points which lie on the plane will be put in here
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // http://answers.ros.org/question/55223/what-does-setoptimizecoefficients-do-in-sacsegmentation/
    // better to set to true if there are likely to be outliers
    seg.setOptimizeCoefficients(true);
    // fitting a plane model to the pointcloud
    seg.setModelType(pcl::SACMODEL_PLANE);
    // use RANSAC algorithm
    seg.setMethodType(pcl::SAC_RANSAC);
    // Points within this distance of the plane are considered as inliers
    seg.setDistanceThreshold(tolerance);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true); // return points which are NOT the indices
    extract.filter(*plane);
    
    return plane;
}

template <typename T>
CloudBounds<T> getCloudBounds(const typename pcl::PointCloud<T>::ConstPtr& cloud){
    T min;
    T max;
    pcl::getMinMax3D(cloud, min, max);
    return CloudBounds<T>(min, max);
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
 *
 * The returned vector contains the planes in descending order of number of
 * points.
 */
// std::vector<pcl::PointCloud<pcl::PointXYZRGB> > extractPlanesByProportion(
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
//     float proportion,
//     float tolerance,
//     Colour colour) {

//     std::vector<pcl::PointCloud<pcl::PointXYZRGB> > extractedPlanes;
//     // Set a boolean if we are to erase 
//     bool erase;
//     if (colour.r == -1 && colour.b == -1 && colour.g == -1){
// 	erase = true;
//     }
    
//     int nExtractedPlanes = 0;
//     int minPointsInPlane = (int) proportion * cloud->points.size();

    
//     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//     pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//     pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    
//     // Create the segmentation object
//     pcl::SACSegmentation<pcl::PointXYZRGB> seg;
//     // Optional
//     seg.setOptimizeCoefficients(true);
//     // Mandatory
//     seg.setModelType(pcl::SACMODEL_PLANE);
//     seg.setMethodType(pcl::SAC_RANSAC);
//     seg.setDistanceThreshold(tolerance);

//     seg.setInputCloud(cloud);
//     seg.segment(*inliers, *coefficients);

//     int nPointsInPlane = inliers->indices.size();
//     std::cerr << "Model #inliers: " << nPointsInPlane << std::endl;
//     if (nPointsInPlane == 0) {
// 	// Couldn't find a plane, so return
//     	PCL_ERROR("Could not estimate a planar model for the given dataset.");
// 	return extractedPlanes;
//     } else if (nPointsInPlane < minPointsInPlane) {
// 	// found a plane, but smaller than minimum proportion, so no point
// 	// continuing.
// 	PCL_INFO("Extracted plane is smaller than minimum proportion.");
// 	return extractedPlanes;
//     }
//     std::cout << "Modifying cloud" << std::endl;

//     extract.setInputCloud(cloud);
//     extract.setIndices(inliers);
//     extract.setNegative(true); // return points which are NOT the indices
//     extract.filter(*plane);

//     if (erase) {
// 	// the cloud now corresponds to all points that were not in the plane.
// 	*cloud = *plane; // using the swap function doesn't work for some reason
//     } else { // Colour all the inliers with the requested colour
// 	PCL_INFO("Colouring points");
// 	for (size_t i = 0; i < inliers->indices.size(); ++i) {
// 	    cloud->points[inliers->indices[i]].r = colour.r;
// 	    cloud->points[inliers->indices[i]].g = colour.g;
// 	    cloud->points[inliers->indices[i]].b = colour.b;
// 	}
//     }
//     return extractedPlanes;
// }

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

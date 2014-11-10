#include "plane_extraction.hpp"

#include <visualization_msgs/Marker.h>

ros::Publisher noplane;
ros::Publisher plane;
ros::Publisher exobj;
ros::Publisher bbox;

/**
 * Subscribing to a PCL type will automatically convert the ROS message type.
 * Need to use a Ptr here, because the setInputCloud requires a ConstPtr. If you
 * use a ConstPtr in the callback, however, you can't modify the contents of the
 * cloud and so need to do some conversions to actually allow modification of
 * the contents.
 */
void pcl_callback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& msg){
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // extract the plane from msg - plane points are removed from msg, present in domPlane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr domPlane = extractDominantPlane(msg, coefficients, 0.02);

    noplane.publish(msg);
    plane.publish(domPlane);

    
    // get the bounds of the plane
    CloudBounds<pcl::PointXYZRGB> domBounds = getCloudBounds<pcl::PointXYZRGB>(domPlane);
    printBounds(domBounds);
    bbox.publish(boundBoxPointCloud(domBounds));

    // Extract points in the remaining cloud which are above or below the plane
    pcl::PointCloud<pcl::PointXYZRGB> onPlanePoints = getPointsInBounds(domBounds, msg, coefficients);

    exobj.publish(onPlanePoints);
}

/**
 * Extracts the plane with the largest number of points in it from the point
 * cloud. The coefficients object will contain its coefficients. The returned
 * cloud will contain the plane, and the cloud passed in will have the plane
 * removed.
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr extractDominantPlane(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    pcl::ModelCoefficients::Ptr coefficients,
    float tolerance) {
    // Stores the coefficients of the plane, defined as
    // ax + by + cz + d = 0
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr notplane(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>());
    // The points which lie on the plane will be put in here
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    // Create the segmentation objectnew pcl::PointCloud<pcl::PointXYZRGB>
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
    extract.filter(*notplane);

    extract.setNegative(false); // return points which ARE the indices
    extract.filter(*plane);

    *cloud = *notplane;

    return plane;
}

/**
 * PrimeSense PCL dimensions: X = horizontal, Y = vertical, Z = depth
 */
pcl::PointCloud<pcl::PointXYZRGB> getPointsInBounds(
    CloudBounds<pcl::PointXYZRGB> bounds,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const pcl::ModelCoefficients::Ptr& coefficients) {
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ProjectInliers<pcl::PointXYZRGB> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*projected);

    // All points in projected now lie in the plane. The points that we are
    // interested in lie in the plane segment defined by the bounding box. Since
    // all points are projected onto the plane, checking for points which lie in
    // the bounding box is sufficient - although the box is in 3D, it will only
    // contain points in the plane segment. There is no risk of points "above"
    // or "below" the plane being set as inside the segment as these will have
    // been projected somewhere to the sides of the plane, whereas without the
    // projection they may have been captured by the bounding box somehow.

    // Points which are in the given bounds are inserted into the final cloud.
    std::vector<int> pointIndices;
    for (size_t i = 0; i < (*projected).size(); i++) {
	if (pointInBounds((*projected)[i], bounds)){
	    //(*filtered).push_back((*projected)[i]);
	    pointIndices.push_back(i);
	}
    }

    return pcl::PointCloud<pcl::PointXYZRGB>(*cloud, pointIndices);
}

bool pointInBounds(pcl::PointXYZRGB point, CloudBounds<pcl::PointXYZRGB> bounds) {
    // short circuit evaluation
    if (point.x < bounds.lower.x || point.x > bounds.upper.x
	|| point.y < bounds.lower.y || point.y > bounds.upper.y
	|| point.z < bounds.lower.z || point.z > bounds.upper.z){
	return false;
    }
    return true;
}

void projTest()
{
    std::cout << "start" << std::endl;
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    coefficients->values.resize (4);
    std::cout << "cloud" << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
    // plane at 10 on the y axis, parallel to XZ plane
    coefficients->values[0] = -0;
    coefficients->values[1] = 1;
    coefficients->values[2] = 0;
    coefficients->values[3] = -10;


    std::cout << "values" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    cloud->width  = 3;
    cloud->height = 1;
    std::cout << "resize" << std::endl;
    cloud->points.resize (cloud->width * cloud->height);
    // expect these points to have y coord of 10 after the projection
    cloud->points[0].x = 1;
    cloud->points[0].y = 2;
    cloud->points[0].z = 3;

    cloud->points[1].x = 2;
    cloud->points[1].y = 0;
    cloud->points[1].z = 1;

    cloud->points[2].x = 10;
    cloud->points[2].y = 20;
    cloud->points[2].z = -5;    

    std::cerr << "Cloud before projection: " << std::endl;
    for (size_t i = 0; i < cloud->points.size (); ++i)
	std::cerr << "    " << cloud->points[i].x << " " 
		  << cloud->points[i].y << " " 
		  << cloud->points[i].z << std::endl;

    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(*cloud_projected);

    std::cerr << "Cloud after projection: " << std::endl;
    for (size_t i = 0; i < cloud_projected->points.size (); ++i)
	std::cerr << "    " << cloud_projected->points[i].x << " " 
		  << cloud_projected->points[i].y << " " 
		  << cloud_projected->points[i].z << std::endl;
}

template <typename T>
CloudBounds<T> getCloudBounds(const typename pcl::PointCloud<T>::ConstPtr& cloud){
    T min;
    T max;
    pcl::getMinMax3D(*cloud, min, max);
    return CloudBounds<T>(min, max);
}

pcl::PointXYZRGB initXYZRGB(float x, float y, float z, int r, int g, int b){
    pcl::PointXYZRGB p;
    p.x = x;
    p.y = y;
    p.z = z;
    p.r = r;
    p.g = g;
    p.b = b;
    
    return p;
}

/**
 * Create a point cloud containing the 8 points which represent the bounding box given
 */
pcl::PointCloud<pcl::PointXYZRGB> boundBoxPointCloud(CloudBounds<pcl::PointXYZRGB> bounds){
    pcl::PointCloud<pcl::PointXYZRGB> bbox;
    bbox.header.frame_id = "camera_link";
    // front face
    bbox.push_back(initXYZRGB(bounds.lower.z, bounds.lower.x, bounds.lower.y, 0, 0, 0));
    bbox.push_back(initXYZRGB(bounds.lower.z, bounds.lower.x, bounds.upper.y, 255, 0, 0));
    bbox.push_back(initXYZRGB(bounds.upper.z, bounds.lower.x, bounds.upper.y, 255, 0, 0));
    bbox.push_back(initXYZRGB(bounds.upper.z, bounds.lower.x, bounds.lower.y, 255, 0, 0));
    // back face
    bbox.push_back(initXYZRGB(bounds.lower.z, bounds.upper.x, bounds.lower.y, 0, 255, 0));
    bbox.push_back(initXYZRGB(bounds.lower.z, bounds.upper.x, bounds.upper.y, 0, 255, 0));
    bbox.push_back(initXYZRGB(bounds.upper.z, bounds.upper.x, bounds.upper.y, 255, 255, 255));
    bbox.push_back(initXYZRGB(bounds.upper.z, bounds.upper.x, bounds.lower.y, 0, 255, 0));

    return bbox;
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
    noplane = handle.advertise<sensor_msgs::PointCloud2>("/pcl_test/plane_removed", 10);
    plane = handle.advertise<sensor_msgs::PointCloud2>("/pcl_test/extracted_plane", 10);
    exobj = handle.advertise<sensor_msgs::PointCloud2>("/pcl_test/objects_plane", 10);
    bbox = handle.advertise<sensor_msgs::PointCloud2>("/pcl_test/bbox", 10);
    
    ros::Rate loop_rate(10);
    while (ros::ok()){
	ros::spinOnce();
	loop_rate.sleep();
    }
}

#include "plane_extraction.hpp"

ros::Publisher pub;
//ros::Publisher plane;
//ros::Publisher exobj;
//ros::Publisher bbox;
//ros::Publisher noplane_organized;
//ros::Publisher coeffs;

using namespace PCLUtil;

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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_org(new pcl::PointCloud<pcl::PointXYZRGB>());


    extractDominantPlane(msg, cloud_org, coefficients, 0.02);
    
    pcl::ModelCoefficients::Ptr tmp(new pcl::ModelCoefficients);
    
    //noplane.publish(msg);
    //plane.publish(domPlane);
    vision_msgs::plane_extracted out_msg;



    sensor_msgs::PointCloud2 ros_nonorg;
    sensor_msgs::PointCloud2 ros_org;
    pcl_msgs::ModelCoefficients ros_coefficients;
    pcl::toROSMsg(*msg,ros_nonorg);
    pcl::toROSMsg(*cloud_org,ros_org);
    //pcl::toROSMsg()
    //pcl_conversions::fromPCL(*msg, ros_nonorg);
    //pcl_conversions::fromPCL(*cloud_org, ros_org);

    out_msg.plane_eq = ros_coefficients;
    out_msg.plane_removed = ros_nonorg;
    out_msg.plane_removed_org = ros_org;
    pub.publish(out_msg);
    //coeffs.publish(ros_coefficients);

    
    // get the bounds of the plane
    /*
    CloudBounds<pcl::PointXYZRGB> domBounds = getCloudBounds<pcl::PointXYZRGB>(domPlane);
    printBounds(domBounds);
    CloudBounds<pcl::PointXYZRGB> scaledBounds = scaleBounds(domBounds, 0.8);
    bbox.publish(boundBoxPointCloud(scaledBounds));

    // Extract points in the remaining cloud which are above or below the plane
    pcl::PointCloud<pcl::PointXYZRGB> onPlanePoints = getPointsInBounds(scaledBounds, msg, coefficients);

    exobj.publish(onPlanePoints);
*/
    //TODO since we allocate memory with new, we need to delete it as well?
    //delete coefficients;
    //delete tmp;

}

/**
 * Extracts the plane with the largest number of points in it from the point
 * cloud. The coefficients object will contain its coefficients. The returned
 * cloud will contain the plane, and the cloud passed in will have the plane
 * removed.
 */
void extractDominantPlane(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_org,
    pcl::ModelCoefficients::Ptr coefficients,
    float tolerance) {
    // Stores the coefficients of the plane, defined as
    // ax + by + cz + d = 0
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr notplane(new pcl::PointCloud<pcl::PointXYZRGB>());
    *cloud_org = *cloud;
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr notplane_org(new pcl::PointCloud<pcl::PointXYZRGB>());
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>());
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

    //extract.setNegative(false); // return points which ARE the indices
    //extract.filter(*plane);

    *cloud = *notplane;
    //ROS_INFO_STREAM("CLOUD_HEIGHT: " << cloud->height);
    //ROS_INFO_STREAM("CLOUD_ORG_HEIGHT: " << cloud_org->height);

    for(int i = 0; i < inliers->indices.size(); ++i) {
        int col = i%cloud_org->width;
        int row = (i-col)/(cloud_org->width);
        cloud_org->at(col,row).z = -100;
    }
    //pcl::PointXYZRGB ppp;




    //return plane;
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

int main (int argc, char* argv[]) {
    ros::init(argc, argv, "plane_extraction");
    ros::NodeHandle handle;
    
    ros::Subscriber depth_cloud = handle.subscribe("/camera/depth_registered/points", 1, pcl_callback);
    // publish a ROS type - can automatically convert from the PCL type
    //noplane = handle.advertise<sensor_msgs::PointCloud2>("/plane_extraction/plane_removed", 10);
    //noplane_organized = handle.advertise<sensor_msgs::PointCloud2>("/plane_extraction/plane_removed_org", 10);
    //plane = handle.advertise<sensor_msgs::PointCloud2>("/plane_extraction/extracted_plane", 10);
    //exobj = handle.advertise<sensor_msgs::PointCloud2>("/plane_extraction/objects_plane", 10);
    //bbox = handle.advertise<sensor_msgs::PointCloud2>("/plane_extraction/bbox", 10);
    //coeffs = handle.advertise<pcl_msgs::ModelCoefficients>("/plane_extraction/plane_coefficients",10);
    pub = handle.advertise<vision_msgs::plane_extracted>("/vision/plane_extraction",5);

    
    ros::Rate loop_rate(10);
    while (ros::ok()){
	ros::spinOnce();
	loop_rate.sleep();
    }
}

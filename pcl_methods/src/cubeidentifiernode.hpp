#ifndef CUBEIDENTIFIERNODE_HPP
#define CUBEIDENTIFIERNODE_HPP

#include <ros/ros.h>
#include <pclutil/pclutil.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/features/normal_3d.h>
#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>

class CubeIdentifierNode {
public:
    void update();
    CubeIdentifierNode(int argc, char* argv[]);

    ros::Subscriber coeffs_subscriber;
    ros::Subscriber pc_subscriber;

private:
    ros::NodeHandle nodeSetup(int argc, char* argv[]);
    void runNode(ros::NodeHandle handle);

    //callbacks
    void pcCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& msg);
    void coeffsCallback(const pcl_msgs::ModelCoefficients::ConstPtr &msg);

    ros::Time t_pc;
    ros::Time t_coeff;
    pcl_msgs::ModelCoefficients::ConstPtr p_coeff;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pc;

    //main algorithms
    //see if a normal is close enough to plane:
    bool closeEnough(const pcl::Normal& reference, const pcl::Normal& other);



};

#endif // CUBEIDENTIFIERNODE_HPP

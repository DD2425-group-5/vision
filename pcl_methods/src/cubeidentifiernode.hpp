#ifndef CUBEIDENTIFIERNODE_HPP
#define CUBEIDENTIFIERNODE_HPP

#include <ros/ros.h>
#include <pclutil/pclutil.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

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
    void pcCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void coeffsCallback(const pcl_msgs::ModelCoefficients::ConstPtr &msg);

    ros::Time t_pc;
    ros::Time t_coeff;
    pcl_msgs::ModelCoefficients::ConstPtr p_coeff;
    sensor_msgs::PointCloud2::ConstPtr p_pc;



};

#endif // CUBEIDENTIFIERNODE_HPP

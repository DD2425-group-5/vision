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
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>

#include <rosutil/rosutil.hpp>
#include <std_msgs/Bool.h>
#include <vision_msgs/colors_detected.h>
#include <vision_msgs/color_status.h>
#include <vision_msgs/color_and_cube.h>
#include <vision_msgs/colors_with_shape_info.h>


class CubeIdentifierNode {
public:
    void update();
    CubeIdentifierNode(int argc, char* argv[]);

    ros::Subscriber coeffs_subscriber;
    ros::Subscriber pc_subscriber;
    ros::Subscriber color_subscriber;
    ros::Publisher cube_publisher;

private:
    ros::NodeHandle nodeSetup(int argc, char* argv[]);
    void runNode(ros::NodeHandle handle);

    void readParams(ros::NodeHandle& n);

    //callbacks
    void pcCallback(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& msg);
    void coeffsCallback(const pcl_msgs::ModelCoefficients::ConstPtr &msg);
    void colorCallback(const vision_msgs::colors_detected::ConstPtr& msg);

    ros::Time t_pc;
    ros::Time t_coeff;
    ros::Time t_color;
    pcl_msgs::ModelCoefficients::ConstPtr p_coeff;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pc;
    vision_msgs::colors_detected::ConstPtr c_colors;


    //main algorithms
    //see if a normal is close enough to plane:
    void downSample(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &input,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output);
    void cropToArea(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output, int minRow, int maxRow,
                                        int minCol, int maxCol);

    bool closeEnough(const pcl::Normal& reference, const pcl::Normal& other, float min_thresh);


    //parameters
    float theta;
    float distance_search_radius;
    int num_similar_vectors_thresh;
    float voxel_grid_x;
    float voxel_grid_y;
    float voxel_grid_z;

    bool check_patric_cube;
    bool check_purple_cube;
    int size_around_color;

    //for optimizations
    float close_enough_thresh;

};

#endif // CUBEIDENTIFIERNODE_HPP

#ifndef PURPLECLASSIFIERNODE_HPP
#define PURPLECLASSIFIERNODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv/highgui.h>
#include <vector>
#include <cmath>
#include <visionutil/visionmodels.hpp>
#include "modelParams.hpp"
#include <rosutil/rosutil.hpp>
#include <visionutil/clustering.hpp>

#define _USE_MATH_DEFINES
#include <math.h>



class PurpleClassifierNode {
public:

    ros::Subscriber rgb_subscriber;
    ros::Publisher classifier_publisher;

    void update();
    PurpleClassifierNode(int argc, char* argv[]);

private:
    ros::Time t_rgb;
    sensor_msgs::Image::ConstPtr img;
    cv::Vec3b purple_rgb;
    cv::Vec3b non_purple_rgb;
    VisionModels::color_model_vardim<double> purple_model;
    double sigma_det;
    std::vector<std::vector<double> > sigma_inv;
    double constant;
    cv::Mat disc_image;
    double r_wp;
    double g_wp;
    double rho_diff;
    double s_min;
    double s_max;
    double c_r;
    double c_g;
    double w2;
    double s_min2;
    double s_max2;

    void rgbCallback(const sensor_msgs::Image::ConstPtr &msg);
    cv_bridge::CvImagePtr convertImage();
    void discriminateImage(cv_bridge::CvImagePtr cv_ptr, std::vector<cv::Vec2i>& purple_points);
    float discriminant(double r, double g);
    float gauss(float x, float mu, float sigma);
    void calc_inv_sigma();
    bool process_pixel(double r, double g, double b);

    int toRgbInt(int i);

    void readModel(ros::NodeHandle& n);
    ros::NodeHandle nodeSetup(int argc, char* argv[]);
    void runNode(ros::NodeHandle handle);
};

#endif // PURPLECLASSIFIERNODE_HPP

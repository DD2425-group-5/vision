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
#include <algorithm>

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
    //std::vector<cv::Vec2i> purple_points;
    VisionModels::color_model_vardim<double> purple_model;
    double sigma_det;
    std::vector<std::vector<double> > sigma_inv;
    double constant;

    //parameters
    double rgb_sum_thresh_max;
    double rgb_sum_thresh_min;
    int blur_size;
    int lines_col;
    int lines_row;
    float thresh_col;
    float thresh_row;

    void readParams(ros::NodeHandle& n);

    static bool contourSort(std::vector<cv::Point> c1, std::vector<cv::Point> c2);
    void rgbCallback(const sensor_msgs::Image::ConstPtr &msg);
    cv_bridge::CvImagePtr convertImage();
    void discriminateImage(const cv::Mat& src, cv::Mat& disc_image);
    float discriminant(double r, double g);
    void calc_inv_sigma();
    bool process_pixel(double r, double g, double b);
    void get_pixel_vector(cv::Mat& src, float thresh, std::vector<cv::Vec2i> &out);
    bool is_object(const std::vector<float>& row_sums, const std::vector<float>& col_sums,
                   float thresh_row,float thresh_col, int lines_col, int lines_row);
    bool is_object_help(const std::vector<float>& vec, float thresh, int lines);

    int toRgbInt(int i);

    void readModel(ros::NodeHandle& n);
    ros::NodeHandle nodeSetup(int argc, char* argv[]);
    void runNode(ros::NodeHandle handle);
};

#endif // PURPLECLASSIFIERNODE_HPP

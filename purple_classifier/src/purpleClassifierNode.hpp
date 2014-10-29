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
#include "modelParams.hpp"

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
    std::vector<cv::Point_<int> > purple_points;
    ModelParams purple_model;
    cv::Mat disc_image;

    void rgbCallback(const sensor_msgs::Image::ConstPtr &msg);
    cv_bridge::CvImagePtr convertImage();
    void discriminateImage(cv_bridge::CvImagePtr cv_ptr);
    float discriminant(float r, float g, float b, float prior, const ModelParams& model);
    float gauss(float x, float mu, float sigma);

    int toRgbInt(int i);

    ros::NodeHandle nodeSetup(int argc, char* argv[]);
    void runNode(ros::NodeHandle handle);
};

#endif // PURPLECLASSIFIERNODE_HPP
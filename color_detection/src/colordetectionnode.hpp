#ifndef COLORDETECTIONNODE_HPP
#define COLORDETECTIONNODE_HPP

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

//std
#include <vector>
#include <string>

//image conversion
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//opencv
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>

//util
#include <visionutil/visionmodels.hpp>
#include <rosutil/rosutil.hpp>

class ColorDetectionNode {
public:
    ColorDetectionNode(int argc, char* argv[]);

private:
    struct color_alg_params {
        VisionModels::color_model_vardim<double> color_model;

        int blur_size;
        int lines_col;
        int lines_row;
        float thresh_col;
        float thresh_row;
        /*
        color_alg_params() {}
        color_alg_params(int blur_size, int lines_col, int lines_row, float thresh_col, float thresh_row) :
            blur_size(blur_size), lines_col(lines_col), lines_row(lines_row), thresh_col(thresh_col),
            thresh_row(thresh_row) {}
        */
    };




    //main stuff, mostly ROS
    ros::Subscriber rgb_subscriber;
    ros::Publisher classifier_publisher;
    ros::Time t_rgb;
    sensor_msgs::Image::ConstPtr camera_img_raw;

    void rgbCallback(const sensor_msgs::Image::ConstPtr &msg);
    ros::NodeHandle nodeSetup(int argc, char* argv[]);
    void runNode(ros::NodeHandle handle);
    void update();

    //models
    std::vector<color_alg_params> models;
    void readModel(ros::NodeHandle n, std::string color_model_name, color_alg_params &cap);



    //image processing
    //convert to RGB
    cv_bridge::CvImagePtr convertImage();

    //convert to rg_chromasity
    void rgb2rgChromasity(const cv::Mat& src, cv::Mat& output);


};

#endif // COLORDETECTIONNODE_HPP

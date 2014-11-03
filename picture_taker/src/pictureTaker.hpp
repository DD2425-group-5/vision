#ifndef PICTURETAKER_HPP
#define PICTURETAKER_HPP

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <cstdlib>
#include <sstream>
#include <time.h>
#include <stdlib.h>
//#include <opencv/highgui.h>

class PictureTakerNode {
public:

    ros::Subscriber rgb_subscriber;

    void update();
    PictureTakerNode(int argc, char* argv[]);

private:
    ros::Time t_camera;
    sensor_msgs::Image::ConstPtr img;

    void rgbCallback(const sensor_msgs::Image::ConstPtr &msg);
    void savePicture();

    cv_bridge::CvImagePtr convertImage();

    ros::NodeHandle nodeSetup(int argc, char* argv[]);
    void runNode(ros::NodeHandle handle);
};

#endif // PICTURETAKER_HPP

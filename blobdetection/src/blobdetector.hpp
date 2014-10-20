#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv/highgui.h>
#include <iostream>
#include <limits>

using namespace cv;

class BlobDetectorNode {
public:
    ros::Subscriber depth_subscriber;
    ros::Publisher blob_publisher;

    void depthCallback(const sensor_msgs::Image::ConstPtr &msg);
    cv_bridge::CvImagePtr convertImage();
    cv_bridge::CvImagePtr normalize(cv_bridge::CvImagePtr cv_ptr);
    vector<KeyPoint> detectBlobs(cv_bridge::CvImagePtr cv_ptr);
    KeyPoint getClosestBlob(vector<KeyPoint> blobs, cv_bridge::CvImagePtr cv_ptr);
    void update();
    BlobDetectorNode(int argc, char* argv[]);

private:
    ros::Time t_depth;
    sensor_msgs::Image::ConstPtr img;
    float maxval;

    ros::NodeHandle nodeSetup(int argc, char* argv[]);
    void runNode(ros::NodeHandle handle);
};


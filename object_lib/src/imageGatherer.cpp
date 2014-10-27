#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv/highgui.h>
#include <stdio.h>

void convertImage(const sensor_msgs::Image::ConstPtr &msg) {
    static int callbackCounter = 0;
    callbackCounter++;
    cv_bridge::CvImagePtr cv_ptr;
    try {
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
	ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
    char fname[20];
    sprintf(fname, "images/prfile_%05d.jpg", callbackCounter);
    ROS_INFO(fname);
    cv::imwrite(fname, cv_ptr->image);
    //ROS_INFO("Callbacks: %d", callbackCounter);
    
    cv::namedWindow("img", CV_WINDOW_AUTOSIZE);
    cv::imshow("img", cv_ptr->image);
    cv::waitKey(1);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_gatherer");
    ros::NodeHandle handle;
    
    ros::Subscriber img_subscriber;
    img_subscriber = handle.subscribe("/camera/rgb/image_raw", 1, convertImage);
    ros::Rate loop_rate(10);
    
    while(ros::ok()){
	ros::spinOnce();
	loop_rate.sleep();
    }

    return 0;
}

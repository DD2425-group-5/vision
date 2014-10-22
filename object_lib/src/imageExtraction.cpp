#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>
#include <iostream>

using namespace cv;


void showImage(std::string fileName) {
    Mat im;
    
    
    im = imread(fileName, CV_LOAD_IMAGE_COLOR);

    if (!im.data) {
	ROS_INFO("Could not load image %s", fileName.c_str());
    }
    
    Mat img;
    resize(im, img, Size(), 0.2, 0.2);
    
    namedWindow("Normal", WINDOW_AUTOSIZE);
    imshow("Normal", img);

    Mat gray;
    cvtColor(img, gray, CV_RGB2GRAY);

    namedWindow("Gray", WINDOW_AUTOSIZE);
    imshow("Gray", gray);

    Mat thresh;
    threshold(gray, thresh, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);

    namedWindow("Thresh", WINDOW_AUTOSIZE);
    imshow("Thresh", thresh);

    imwrite("thresholded.jpg", thresh);
    
    Mat masked;
    img.copyTo(masked, thresh);
    
    namedWindow("mask", WINDOW_AUTOSIZE);
    imshow("mask", masked);

    imwrite("mask.jpg", masked);
    
    waitKey(0);
}

int main(int argc, char *argv[]) {
    for (int i = 0; i < argc; i++) {
	ROS_INFO("Arg %d: %s", i, argv[i]);
    }
    showImage(argv[1]);
}

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>
#include <iostream>
#include <algorithm>

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

RNG rng(12345);

bool contourSort(std::vector<Point> c1, std::vector<Point> c2) {
    return contourArea(c1) > contourArea(c2);
}

void detectBlob(std::string fileName) {
    Mat im;
    
    im = imread(fileName, CV_LOAD_IMAGE_COLOR);

    if (!im.data) {
	ROS_INFO("Could not load image %s", fileName.c_str());
    }
    
    Mat img;
    resize(im, img, Size(), 0.2, 0.2);
    Mat gray;
    cvtColor(img, gray, CV_RGB2GRAY);
    Mat thresh;
    threshold(gray, thresh, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);
    
    std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    
    findContours(thresh, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    std::sort(contours.begin(), contours.end(), contourSort);

    Mat drawing = Mat::zeros(img.size(), CV_8UC1);
    Scalar colour = Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
    drawContours(drawing, contours, 0, colour, CV_FILLED);
    
    // for( int i = 0; i< contours.size(); i++ ) {
    // 	if (contourArea(contours[i]) < 500)
    // 	    continue;
    // 	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
    // 	drawContours(img, contours, i, color, 2, 8, hierarchy, 0, Point());
    // }
    
    Mat extracted;
    img.copyTo(extracted, drawing);

    namedWindow("blobextract", CV_WINDOW_AUTOSIZE);
    imshow("blobextract", extracted);


    waitKey(0);
}

int main(int argc, char *argv[]) {
    for (int i = 0; i < argc; i++) {
    	ROS_INFO("Arg %d: %s", i, argv[i]);
    }
    detectBlob(argv[1]);
}

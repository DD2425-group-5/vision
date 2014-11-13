#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>
#include <iostream>

using namespace cv;

Mat src;
int curThresh = 0;
int maxThresh = 255;

/** @function thresh_callback */
void thresh_callback(int, void* ) {
    Mat thresh;
    threshold(src, thresh, (double)curThresh, (double)255, THRESH_BINARY_INV);    
    /// Show in a window
    imshow("src", thresh);
}

int main(int argc, char *argv[]) {
    Mat m = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    resize(m, src, Size(), 0.2, 0.2);
    namedWindow("src", CV_WINDOW_AUTOSIZE);
    imshow("src", src);

    createTrackbar("Threshold", "src", &curThresh, maxThresh, thresh_callback);
    thresh_callback( 0, 0 );

    waitKey(0);
    return(0);
}

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv/highgui.h>
#include <vector>
#include <cmath>
#include <utility>
#include <algorithm>
#include <iostream>
#include <blobdetection/depth_point.h>
#include <rosutil/rosutil.hpp>

using namespace cv;

class ClosestPixelDetectorNode {
public:
    template <typename T>
    struct DepthPoint {
    	T value;
    	int x;
    	int y;
	
	DepthPoint(){}
    	DepthPoint(int xx, int yy, T val){
	    value = val;
	    x = xx;
	    y = yy;
	}
	
	/**
	 * Comparison operator. Allows comparison to be made between two DepthPoints.
	 * This will have the smaller values end up at the start of the array if used
	 * as a parameter to a sorting function.
	 * DepthPoint<float>() is how it should be passed to a function.
	 */
	bool operator() (const DepthPoint& a, const DepthPoint& b) {
	    return a.value < b.value;
	}

	friend std::ostream& operator<<(std::ostream& out, const DepthPoint<T>& pt) {
	    out << "(" << pt.x << ", " << pt.y<< ", " << pt.value << ")";
	    return out;
	}
	
    };

    ros::Subscriber depth_subscriber;
    ros::Publisher depth_point_publisher;

    void depthCallback(const sensor_msgs::Image::ConstPtr &msg);
    cv_bridge::CvImagePtr convertImage();
    cv_bridge::CvImagePtr normalize(cv_bridge::CvImagePtr cv_ptr);
    KeyPoint naiveDetection(cv_bridge::CvImagePtr cv_ptr, int num_closest_pixels);
    void update();
    ClosestPixelDetectorNode(int argc, char* argv[]);
    cv_bridge::CvImagePtr convertImageToRange(cv_bridge::CvImagePtr imgPtr);
    DepthPoint<float> naiveDetectionNthElement(cv_bridge::CvImagePtr imgPtr, int nClosest, bool ignoreZeros);
    template<typename U> std::vector< DepthPoint<U> > cvMatToVector(cv::Mat matrix,bool ignoreZeros);
    void drawFloatImg(cv_bridge::CvImagePtr imgPtr, DepthPoint<float> point);

private:
    ros::Time t_depth;
    sensor_msgs::Image::ConstPtr img;
    float maxval;
    
    float mean_img;
    float std_img;

    //parameters
    int numClosestPixels;
    bool drawImages;
    bool debugMessages;

    void runNode();
};

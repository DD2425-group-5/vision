#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>
#include <vector>
#include <limits>

ros::Subscriber depth_subscriber;
ros::Publisher blob_publisher;

/*
void histogram(cv::Mat img) {
    int histSize = 256;
    float hranges[] = {0, 256};
    int channels[] = {0};
    
    const float* ranges[] = {hranges};
    cv::MatND hist;

    std::vector<cv::Mat> cl;
    cv::split(img, cl);

    calcHist( &cl[0], 1, channels, cv::Mat(), // do not use mask
	      hist, 2, &histSize, ranges,
	      true, // the histogram is uniform
	      false );
    double maxVal=0;
    minMaxLoc(hist, 0, &maxVal, 0, 0);

    int scale = 10;
    cv::Mat histImg = cv::Mat::zeros(histSize*scale, histSize*scale, CV_8UC3);

    for( int h = 0; h < histSize; h++ )
	for( int s = 0; s < histSize; s++ )
	{
	    float binVal = hist.at<float>(h, s);
	int intensity = cvRound(binVal*255/maxVal);
	cv::rectangle( histImg, cv::Point(h*scale, s*scale),
		       cv::Point( (h+1)*scale - 1, (s+1)*scale - 1),
		       cv::Scalar::all(intensity),
		       CV_FILLED );
	}

    cv::namedWindow( "Source", 1 );
    cv::imshow( "Source", img );

    cv::namedWindow( "Hist", 1 );
    cv::imshow( "Hist", histImg );
    cv::waitKey();
}
*/

void histogram(cv::Mat img)
{
    float min = std::numeric_limits<float>::max();
    float max = std::numeric_limits<float>::min();
    int lzeroc = 0;
    
    for (int row = 0; row < img.rows; row++) {
	const float* rowptr = img.ptr<float>(row); // extract row of the matrix
	// create depthpoints and insert them into the flattened vector.
	for (int col = 0; col < img.cols; col++) {
	    if (rowptr[col] < min)
		min = rowptr[col];
	    if (rowptr[col] > max)
		max = rowptr[col];
	    if (rowptr[col] <= 0)
		lzeroc++;
	}
    }

    int nbins = 256;
    float rangeSkip = (max - min)/nbins;
    std::cout << rangeSkip << std::endl;
    std::vector<float> binRanges;
    binRanges.push_back(min);
    for (int i = 1; i < nbins; i++) {
	binRanges.push_back(binRanges[i - 1] + rangeSkip);
    }

    std::vector<int> hist(nbins);
    
    for (int row = 0; row < img.rows; row++) {
	const float* rowptr = img.ptr<float>(row); // extract row of the matrix
	// create depthpoints and insert them into the flattened vector.
	for (int col = 0; col < img.cols; col++) {
	    for (int bin = 1; bin < nbins; bin++) {
		if (rowptr[col] <= binRanges[bin]){
		    hist[bin - 1]++;
		    break;
		}
	    }

	}
    }

    for (int i = 0; i < nbins; i++) {
	std::cout << i << "th bin: " << hist[i] << std::endl;
    }
    
    std::cout << "max is " << max << ", min is " << min << ", lzero is " << lzeroc << std::endl;
    std::exit(0);
    
}
    
void depthCallback(const sensor_msgs::Image::ConstPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
	ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    histogram(cv_ptr->image);

    cv::imshow("DepthImage", cv_ptr->image);
    cv::waitKey(3);
    std::cout << "callback" << std::endl;
}

ros::NodeHandle nodeSetup(int argc, char* argv[]) {
    ros::init(argc, argv, "closestPixelDetector");
    ros::NodeHandle handle;
    cv::namedWindow("DepthImage");
    depth_subscriber = handle.subscribe("/camera/depth/image_raw", 1, depthCallback);
    return handle;
}


	
void runNode(ros::NodeHandle handle) {
    // Control @ 10 Hz
    double control_frequency = 10.0;

    ros::Rate loop_rate(control_frequency);

    while(ros::ok()) {
	ros::spinOnce();
	loop_rate.sleep();
    }
}

int main(int argc, char* argv[])
{
    ros::NodeHandle handle = nodeSetup(argc, argv);
    runNode(handle);
}

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <limits>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
  
public:
    ImageConverter()
    : it_(nh_) {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/depth/image_raw", 1, &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
	ROS_INFO("Started image converter.");
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

	double min = std::numeric_limits<double>::max();
	double max = std::numeric_limits<double>::min();
	int count = 0;
	for (int i = 0; i < cv_ptr->image.rows; i++)
	{
	    const double* Mi = cv_ptr->image.ptr<double>(i);
	    for(int j = 0; j < cv_ptr->image.cols; j++){
		// if (j = 0){
		//     ROS_INFO_STREAM("" << Mi[j]);
		// }
		// if (Mi[j] > 1e150){
		//     count++;
		//      cv_ptr->image.at<double>(i, j) = 0;
		//      continue;
		// }
		
		if (Mi[j] < min)
		    min = Mi[j];
		if (Mi[j] > max)
		    max = Mi[j];
	    }
		
	}

	ROS_INFO_STREAM("Max is " << max);
	ROS_INFO_STREAM("Min is " << min);
	ROS_INFO_STREAM("Reject count " << count);

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);
    
        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imageConverter");
    ImageConverter ic;
    ros::spin();
    return 0;
}

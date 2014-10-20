#include "blobDetector.hpp"

void BlobDetectorNode::depthCallback(const sensor_msgs::Image::ConstPtr &msg) {
    t_depth = ros::Time::now();
    img = msg;
}
    
cv_bridge::CvImagePtr BlobDetectorNode::convertImage() {
    cv_bridge::CvImagePtr cv_ptr;
    try {
	cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e) {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return cv_ptr;
    }

    /**
     * Find min and max values of the image to use in normalisation. Using the
     * float references is important, otherwise the values that come out are
     * really weird
     */
    float min = std::numeric_limits<float>::max();
    float max = std::numeric_limits<float>::min();
    int count = 0;
    for (int i = 0; i < cv_ptr->image.rows; i++) {
	const float* Mi = cv_ptr->image.ptr<float>(i);
	for(int j = 0; j < cv_ptr->image.cols; j++){
	    if (Mi[j] < min)
		min = Mi[j];
	    if (Mi[j] > max)
		max = Mi[j];
	}
    }

    maxval = max;
    //ROS_INFO_STREAM("Min: " << min << ", Max: " << max << ", Count: " << count);

    /**
     * Again, need to make sure that at<float> is used otherwise weird stuff
     * happens Need to put values into the range 0-255, otherwise the conversion
     * afterwards will not have the desired result.
     */
    
    for (int i = 0; i < cv_ptr->image.rows; ++i) {
	for(int j = 0; j < cv_ptr->image.cols; ++j) {
	    cv_ptr->image.at<float>(i,j) =  (cv_ptr->image.at<float>(i,j) / max)*255;
	}
    }

    /**
     * Need to convert to CV_8UC1, otherwise the SimpleBlobDetector doesn't work, 
     * crashes at either the thresholding or contour stage.
     */
    Mat m(img->height,img->width,CV_8UC1);
    cv_ptr->image.convertTo(m,CV_8UC1);
    cv_ptr->image = m;

    return cv_ptr;
}
    
cv_bridge::CvImagePtr BlobDetectorNode::normalize(cv_bridge::CvImagePtr cv_ptr) {
    return cv_ptr;
}
    
/*Detect blobs from the stored image*/
vector<KeyPoint> BlobDetectorNode::detectBlobs(cv_bridge::CvImagePtr cv_ptr) {
    // set up the parameters (check the defaults in opencv's code in blobdetector.cpp)
    cv::SimpleBlobDetector::Params params;
    params.minDistBetweenBlobs = 10.0f;
    params.filterByInertia = false;
    params.filterByConvexity = false;
    params.filterByColor = false;
    params.filterByCircularity = false;
    params.filterByArea = true;
    params.minArea = 2500.0f;
    params.maxArea = 400000.0f;
    // ... any other params you don't want default value

    // set up and create the detector using the parameters
    cv::Ptr<cv::FeatureDetector> blob_detector = new cv::SimpleBlobDetector(params);
    blob_detector->create("SimpleBlob");

    // detect!
    vector<cv::KeyPoint> keypoints;
    blob_detector->detect(cv_ptr->image,keypoints);
    return keypoints; //TODO
}
    
KeyPoint BlobDetectorNode::getClosestBlob(vector<KeyPoint> blobs, cv_bridge::CvImagePtr cv_ptr) {
    int min = -2000;
    int minindex = 0;
    KeyPoint kp;
    if(blobs.size() == 0) {
	kp.pt.x = -1;
	kp.pt.y = -1;
	return kp;
    }

    std::cout << "blobs found: " << blobs.size() << std::endl;

    for(int i = 0; i < blobs.size();++i) {
	//ROS_INFO("x: ",blobs[i].pt.x," , y: ", blobs[i].pt.y );
	char tmpa = cv_ptr->image.at<char>(blobs[i].pt.y,blobs[i].pt.x);
	//std::cout << (unsigned int)tmpa << std::endl;
	int tmp = (int) tmpa;
	if(tmp > min) {
	    min = tmp;
	    minindex = i;
	}
    }

    cv::circle(cv_ptr->image, cv::Point(blobs[minindex].pt.x, blobs[minindex].pt.y),
	       blobs[minindex].size/2, CV_RGB(255,255,255));
    cv::imshow("DepthImage",cv_ptr->image);
    cv::waitKey(3);

    kp.pt.x = blobs[minindex].pt.x;
    kp.pt.y = min;
    return kp;
}
void BlobDetectorNode::update() {
    // if more than 2 seconds have passed and no messages have been received,
    // stop sending
    if((ros::Time::now()-t_depth).toSec()>1.0) {
	return;
    }
        
    //convert image to openCV image
    cv_bridge::CvImagePtr cv_ptr = convertImage();
    //normalize image, remove max values
    //    cv_ptr = normalize(cv_ptr);
    //if(1) return;

    //detect blobs in image
    vector<KeyPoint> points = detectBlobs(cv_ptr);
    //pick the closest blob
    KeyPoint kp = getClosestBlob(points,cv_ptr);
    
    ROS_INFO_STREAM("x cor: " << kp.pt.x << " distance: " << kp.pt.y);

    // calculate kinematics and send twist to robot simulation node
    geometry_msgs::Twist twist_msg;

    twist_msg.linear.x = kp.pt.x;
    twist_msg.linear.y = 0.0;
    twist_msg.linear.z = kp.pt.y;

    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = 0.0;

    blob_publisher.publish(twist_msg);
}


ros::NodeHandle BlobDetectorNode::nodeSetup(int argc, char* argv[]) {
    ros::init(argc, argv, "blobDetector");
    ros::NodeHandle handle;
    t_depth = ros::Time::now();
    cv::namedWindow("DepthImage");
    depth_subscriber = handle.subscribe("/camera/depth/image_raw", 1, &BlobDetectorNode::depthCallback, this);
    blob_publisher = handle.advertise<geometry_msgs::Twist>("/vision/blobdetection", 1);
    return handle;
}
    
	
void BlobDetectorNode::runNode(ros::NodeHandle handle) {
    // Control @ 10 Hz
    double control_frequency = 10.0;

    ros::Rate loop_rate(control_frequency);
    for(int i = 0; i < 10; ++i)
	loop_rate.sleep();


    while(ros::ok()) {
	update();
	ros::spinOnce();
	loop_rate.sleep();
    }
}

BlobDetectorNode::BlobDetectorNode(int argc, char* argv[]) {
    ros::NodeHandle handle = nodeSetup(argc, argv);
    runNode(handle);
}

int main(int argc, char* argv[])
{
    BlobDetectorNode bdn(argc, argv);
}

#include "blobdetector.hpp"

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
    return cv_ptr; //TODO
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
    cv::imshow("windowname",cv_ptr->image);
    cv::waitKey(3);

    kp.pt.x = blobs[minindex].pt.x;
    kp.pt.y = min;
    return kp;
}

//A struct used for sorting in the naiveDecetion algorithm
//initialize an object of this struct with an image and input
//it to the sort function. Sorts by how close the points are.
//in ascending or descending? I can never remember this...
//NOTE: not used in the current algorithm. Can probably remove.
struct comp {
    cv_bridge::CvImagePtr cv_ptr;
    
    comp() {}
    comp(cv_bridge::CvImagePtr image_pointer) {
        cv_ptr = image_pointer;
    }
    
    bool operator() (const KeyPoint& kp1, const KeyPoint& kp2) {
        int row1 = (int) kp1.pt.y;
        int col1 = (int) kp1.pt.x;
        int row2 = (int) kp2.pt.y;
        int col2 = (int) kp2.pt.x;
        return cv_ptr.at<float>(row1,col1) < cv_ptr.at<float>(row2,col2);
    }
};

/* Some code for own naive blob detection. Take the num_closest_pixels pixels and just assume that
is the blob. 

Complexity of this algorithm: width*height*num_closest_pixels.
*/
KeyPoint BlobDetectorNode::naiveDetection(cv_bridge::CvImagePtr cv_ptr, int num_closest_pixels) {
    std::vector<KeyPoint> closest_points;
    
    for (int i = 0; i < cv_ptr->image.rows; ++i) {
	for (int j = 0; j < cv_ptr->image.cols; ++j) {
        if(closest_point.size() < num_closest_pixels) {
            //for the first points, just append to the vector
            KeyPoint kp;
            kp.x = j;
            kp.y = i;
            closest_points.push_back(kp);
        } else {
            //when we have num_closest_points in the vector
            //find the point furthest away and replace it with this one.
            KeyPoint minkp;
            int minindex = -1;
            for(int k = 0; k < closest_point.size(),++k) {
                if(cv_ptr.at<float>(j,i) < cv_ptr.at<float>(closest_points[k].pt.y,closest_points[k].pt.x)) {
                    minindex = k;
                    minkp.pt.y = i;
                    minkp.pt.x = j;
                }
            }
            
            if(minindex != -1) {
                //means we found a point further away.
                closest_points[minindex].x = minkp.x;
                closest_points[minindex].y = minkp.y;
            }
        }
	}
    }
    
    //now we have a vector of keypoints of size num_closest_pixels
    //they contain the pixels with the smallest depth to the camera.
    //Note: no normalization is done here. This assumes a normalized image.
    //I don't know how necessary the normalization is but its pretty important
    //if we get a bunch of distances that are just 0.
    //normalization of image I: I_n = (I-mean(I))/std(I), where std is the standard deviation.
    
    //so now find out average depth, and average position.
    //to make it all cleaner perhaps we should consider just having a struct with 3 values:
    //int row, int col, float depth.
    float depth_mean = 0;
    int x_mean = 0;
    //do not calculate average y, we don't need that.
    
    for(int i = 0; i < closest_points.size(); ++i) {
        depth_mean += cv_ptr.at<float>(closest_points[i].pt.y, closest_points[i].pt.x);
        x_mean += closest_points[i];
    }
    
    x_mean = x_mean / ((int) closest_points.size()); //integer division, should be ok.
    depth_mean = depth_mean / ((float) closest_points.size());
    
    KeyPoint kp; //such abuse of KeyPoint... we really should make our own structs.
    kp.x = x_mean;
    kp.y = depth_mean;
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
    cv_ptr = normalize(cv_ptr);
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
    cv::namedWindow("windowname");
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

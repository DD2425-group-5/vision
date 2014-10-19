#include "closestPixelDetector.hpp"

void ClosestPixelDetectorNode::depthCallback(const sensor_msgs::Image::ConstPtr &msg) {
    t_depth = ros::Time::now();
    img = msg;
}
    
cv_bridge::CvImagePtr ClosestPixelDetectorNode::convertImage() {
    cv_bridge::CvImagePtr cv_ptr;
    try {
	cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e) {
	ROS_ERROR("cv_bridge exception: %s", e.what());
	return cv_ptr;
    }
    return cv_ptr;
}
    
cv_bridge::CvImagePtr ClosestPixelDetectorNode::normalize(cv_bridge::CvImagePtr cv_ptr) {
    //normalization of image I: I_n = (I-mean(I))/std(I), where std is the standard deviation.
    //std definition: http://www.mathworks.se/help/matlab/ref/std.html
    //using nr 1 here.
    //using all floats here because the image is in float format.
    
    //TODO: probably faster to iterate via iterators, but I'm not sure of syntax...
    
    float mean = 0;
    float std = 0;
    float size = ((float)cv_ptr->image.rows) * ((float)cv_ptr->image.cols);
    
    //calculate mean
    for (int i = 0; i < cv_ptr->image.rows; ++i) {
	for (int j = 0; j < cv_ptr->image.cols; ++j) {
        mean += cv_ptr->image.at<float>(i,j);
    }
    }
    
    mean = mean/size;
    mean_img = mean; //save as class variable to be able to reverse normalization.
    
    //calculate std
    for (int i = 0; i < cv_ptr->image.rows; ++i) {
	for (int j = 0; j < cv_ptr->image.cols; ++j) {
        std += std::pow(cv_ptr->image.at<float>(i,j) - mean,2);
    }
    }

    std = std / (size-1);
    std = std::sqrt(std);
    std_img = std;
    
    //normalize image
    for (int i = 0; i < cv_ptr->image.rows; ++i) {
	for (int j = 0; j < cv_ptr->image.cols; ++j) {
        float val = cv_ptr->image.at<float>(i,j);
        cv_ptr->image.at<float>(i,j) = (val - mean)/std; //not sure if this works.
        //basically: pixel = (pixel-mean)/std needs to be done.
    }
    }
    
    return cv_ptr;
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
        return cv_ptr->image.at<float>(row1,col1) < cv_ptr->image.at<float>(row2,col2);
    }
};

/* Some code for own naive blob detection. Take the num_closest_pixels pixels and just assume that
is the blob. 

Complexity of this algorithm: width*height*num_closest_pixels.
*/
KeyPoint ClosestPixelDetectorNode::naiveDetection(cv_bridge::CvImagePtr cv_ptr, int num_closest_pixels) {
    std::vector<KeyPoint> closest_points;
    for (int i = 0; i < cv_ptr->image.rows; ++i) {
	for (int j = 0; j < cv_ptr->image.cols; ++j) {
        if(closest_points.size() < num_closest_pixels) {
            //for the first points, just append to the vector
            KeyPoint kp;
            kp.pt.x = j;
            kp.pt.y = i;
            closest_points.push_back(kp);
        } else {
            //when we have num_closest_points in the vector
            //find the point furthest away and replace it with this one.
            KeyPoint minkp;
            int minindex = -1;
            for(int k = 0; k < closest_points.size(); ++k) {
                if(cv_ptr->image.at<float>(i,j) < cv_ptr->image.at<float>(closest_points[k].pt.y,closest_points[k].pt.x)) {
                    minindex = k;
                    minkp.pt.y = i;
                    minkp.pt.x = j;
                }
            }
            
            if(minindex != -1) {
                //means we found a point further away.
                closest_points[minindex].pt.x = minkp.pt.x;
                closest_points[minindex].pt.y = minkp.pt.y;
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

    // Create an image where indices in the closest points vector are white, all others black.
    cv_bridge::CvImagePtr blobImage;
    
    std::cout << cv_ptr->image.rows << ":" << cv_ptr->image.cols << std::endl;

    //do not calculate average y, we don't need that.    
    for(int i = 0; i < closest_points.size(); ++i) {
        depth_mean += cv_ptr->image.at<float>(closest_points[i].pt.y, closest_points[i].pt.x);
//	blobImage->image.at<float>(closest_points[i].pt.y, closest_points[i].pt.x) = 255;
        x_mean += closest_points[i].pt.x;
    }

    cv::Mat m;
    blobImage->image.convertTo(m, CV_8UC1);
    blobImage->image = m;
        
    std::cout << "imshow" << std::endl;
    cv::imshow("BlobImage", cv_ptr->image);
    std::cout << "after imshow" << std::endl;
    x_mean = x_mean / ((int) closest_points.size()); //integer division, should be ok.
    depth_mean = depth_mean / ((float) closest_points.size());
    
    KeyPoint kp; //such abuse of KeyPoint... we really should make our own structs.
    kp.pt.x = x_mean;
    kp.pt.y = depth_mean;
    return kp;
}
    

void ClosestPixelDetectorNode::update() {
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

    KeyPoint kp = naiveDetection(cv_ptr, 150);
    
    //unnormalize depth.
    kp.pt.y = (kp.pt.y * std_img) + mean_img;

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


ros::NodeHandle ClosestPixelDetectorNode::nodeSetup(int argc, char* argv[]) {
    ros::init(argc, argv, "closestPixelDetector");
    ros::NodeHandle handle;
    t_depth = ros::Time::now();
    cv::namedWindow("BlobImage");
    depth_subscriber = handle.subscribe("/camera/depth/image_raw", 1, &ClosestPixelDetectorNode::depthCallback, this);
    blob_publisher = handle.advertise<geometry_msgs::Twist>("/vision/pixeldetection", 1);
    return handle;
}
    
	
void ClosestPixelDetectorNode::runNode(ros::NodeHandle handle) {
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

ClosestPixelDetectorNode::ClosestPixelDetectorNode(int argc, char* argv[]) {
    ros::NodeHandle handle = nodeSetup(argc, argv);
    runNode(handle);
}

int main(int argc, char* argv[])
{
    ClosestPixelDetectorNode bdn(argc, argv);
}

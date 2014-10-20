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

cv_bridge::CvImagePtr ClosestPixelDetectorNode::convertImageToRange(cv_bridge::CvImagePtr imgPtr) {
    /**
     * Find min and max values of the image to use in normalisation. Using the
     * float references is important, otherwise the values that come out are
     * really weird
     */
    float min = std::numeric_limits<float>::max();
    float max = std::numeric_limits<float>::min();
    int count = 0;
    for (int i = 0; i < imgPtr->image.rows; i++) {
	const float* Mi = imgPtr->image.ptr<float>(i);
	for(int j = 0; j < imgPtr->image.cols; j++){
	    if (Mi[j] < min)
		min = Mi[j];
	    if (Mi[j] > max)
		max = Mi[j];
	}
    }

    float maxval = max;
    //ROS_INFO_STREAM("Min: " << min << ", Max: " << max << ", Count: " << count);

    /**
     * Again, need to make sure that at<float> is used otherwise weird stuff
     * happens Need to put values into the range 0-255, otherwise the conversion
     * afterwards will not have the desired result.
     */
    
    for (int i = 0; i < imgPtr->image.rows; ++i) {
	for(int j = 0; j < imgPtr->image.cols; ++j) {
        imgPtr->image.at<float>(i,j) = ( imgPtr->image.at<float>(i,j) / maxval)*255.0f;
	}
    }

    /**
     * Need to convert to CV_8UC1, otherwise the SimpleBlobDetector doesn't work, 
     * crashes at either the thresholding or contour stage.
     */
    Mat m(imgPtr->image.rows,imgPtr->image.cols,CV_8UC1);
    imgPtr->image.convertTo(m,CV_8UC1);
    imgPtr->image = m;

    
    return imgPtr;
}

/**
 * Flattens a cv::Mat into a vector, stacking the rows. The template should
 * specify the type of data inside the matrix, just like all other cases when
 * referring to data inside it. Returns a struct with fields x,y and value. The
 * row is inserted into y, column into x.
 */
template <typename T>
std::vector< ClosestPixelDetectorNode::DepthPoint<T> > ClosestPixelDetectorNode::cvMatToVector(cv::Mat matrix,bool ignoreZeros) {
    std::vector< ClosestPixelDetectorNode::DepthPoint<T> > flattened;
    
    // populate the flattened vector
    for (int row = 0; row < matrix.rows; row++) {
        const T* rowptr = matrix.ptr<T>(row); // extract row of the matrix
        // create depthpoints and insert them into the flattened vector.
        for (int col = 0; col < matrix.cols; col++) {
            if(ignoreZeros && rowptr[col] <= 1e-10)
                continue;
            flattened.push_back(ClosestPixelDetectorNode::DepthPoint<T>(col, row, rowptr[col]));
        }
    }

    return flattened;
}

/* Some code for own naive blob detection. Take the num_closest_pixels pixels and just assume that
is the blob. 

Complexity of this algorithm: width*height*num_closest_pixels. -> takes way too long to get anything out. Tried with 500 closest points, and takes ~1sec to process a single image.

Use built in cpp nth_element? Should be able to extract the relevant values from
the resulting iterators. Might need to flatten the matrix to do this, though.
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
                if(cv_ptr->image.at<float>(i,j) < cv_ptr->image.at<float>(
                            closest_points[k].pt.y,closest_points[k].pt.x)) {
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

    // Create a matrix where indices in the closest points vector are white, all others black.
    cv::Mat m = cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);

    //do not calculate average y, we don't need that.    
    for(int i = 0; i < closest_points.size(); ++i) {
	float y = closest_points[i].pt.y;
	float x = closest_points[i].pt.x;
	//std::cout << i << ": (" << x << ", " << y << ")" << std::endl;
        depth_mean += cv_ptr->image.at<float>(y, x);
	m.at<float>(y, x) = 255;
        x_mean += x;
    }
    
    cv::imshow("DepthImage", cv_ptr->image);
    cv::waitKey(3);

    cv::imshow("BlobImage", m);
    cv::waitKey(3);
    
    x_mean = x_mean / ((int) closest_points.size()); //integer division, should be ok.
    depth_mean = depth_mean / ((float) closest_points.size());
    
    KeyPoint kp; //such abuse of KeyPoint... we really should make our own structs.
    kp.pt.x = x_mean;
    kp.pt.y = depth_mean;
    return kp;
}

/**
 * Find the average point of the nClosest closest points in the given image.
 * First, flattens the image matrix into a vector, and then uses the built in
 * function nth_element to rearrange array elements in such a way that all
 * elements below a certain index have a value lower or equal to the value at
 * that index. Those elements are out of order, but we don't care about ordering
 * since we need a mean. 
 *
 * The ignoreZeros flag specifies whether the function should take into account
 * zeros in the image. If false, the mean will be computed using a mix of valid
 * and invalid points (assuming that all points with zero depth are invalid).
 * Otherwise, all points used in the computation will be valid.
 */
ClosestPixelDetectorNode::DepthPoint<float> ClosestPixelDetectorNode::naiveDetectionNthElement(
        cv_bridge::CvImagePtr imgPtr, int nClosest, bool ignoreZeros)
{
    // Flatten the image matrix into a vector so that it can be used in the next stage
    std::vector< DepthPoint<float> > flat = cvMatToVector<float>(imgPtr->image,ignoreZeros);

    /**
     * Use nth_element to order the DepthPoints in the vector in such a way that
     * the element at flat[nClosest] is the one that would be in that position
     * in a sorted array. That is, in the resulting vector, flat[nClosest] is
     * the only one that is correctly sorted. The other elements are left
     * without any specific order, except that none of the elements preceding
     * nth are greater than it, and none of the elements following it are less.
     *
     * Thus, we can be sure that the depth of any element at any index below
     * nClosest is <= to the depth of nClosest itself. Since we don't care about
     * any particular ordering, we can exploit this fact by simply taking the
     * mean of lower-index elements to get our "blob" centre.
     */
    std::nth_element(flat.begin(), flat.begin() + nClosest, flat.end(), DepthPoint<float>());
    // Store information about the closest points in here
    //cv::Mat blobMat = cv::Mat::zeros(imgPtr->image.rows, imgPtr->image.cols, CV_32F);
    cv::Mat blobMat = cv::Mat::zeros(imgPtr->image.rows, imgPtr->image.cols, CV_8UC1);

    int xSum = 0;
    int ySum = 0;
    float depthSum = 0;
    int counted = 0;
    int totalElements = imgPtr->image.cols * imgPtr->image.rows;
    for (int i = 0; i < nClosest; i++) {
        // Perhaps these if statements could be improved. I think branch
        // prediction mitigates most of the possible slowdown if done like this.
        if (!ignoreZeros){ // if counting zeros, just add all values
            counted++;
            xSum += flat[i].x;
            ySum += flat[i].y;
            depthSum += flat[i].value;
            blobMat.at<char>(flat[i].y, flat[i].x) = 255;
        } else {
            // need to exclude zeros and count nonzero values to get correct mean
            if (flat[i].value <= 1e-10) {
                    // Need to loop an additional time for each zero
            continue;
            }
            counted++;
            xSum += flat[i].x;
            ySum += flat[i].y;
            depthSum += flat[i].value;
            blobMat.at<char>(flat[i].y, flat[i].x) = 255;
            // std::cout << " nonzero";
            // xSum += flat[i].x;
            // std::cout << " x added";
            // ySum += flat[i].y;
            // std::cout << " y added";
            // depthSum += flat[i].value;
            // std::cout << "depth added";
            // blobMat.at<int>(flat[i].y, flat[i].x) = 255;
            // std::cout << " mat updated" << std::endl;

        }
    }
    if(drawImages)
        cv::imshow("BlobImage", blobMat);
    return DepthPoint<float>(xSum/counted, ySum/counted, depthSum/ (float)counted);
}

void ClosestPixelDetectorNode::drawFloatImg(cv_bridge::CvImagePtr imgPtr,DepthPoint<float> point) {
    imgPtr = convertImageToRange(imgPtr);
    cv::circle(imgPtr->image, cv::Point(point.x, point.y),
           15, CV_RGB(255,255,255));
    cv::imshow("DepthImage", imgPtr->image);
    cv::waitKey(3);
}


    

void ClosestPixelDetectorNode::update() {
    // if more than 2 seconds have passed and no messages have been received,
    // stop sending
    if((ros::Time::now()-t_depth).toSec()>1.0) {
	return;
    }
        
    //convert image to openCV image
    cv_bridge::CvImagePtr cv_ptr = convertImage();
    
    //cv_ptr = normalize(cv_ptr);
    //cv_ptr = convertImageToRange(cv_ptr);
    DepthPoint<float> closestPixelsMean = naiveDetectionNthElement(cv_ptr, numClosestPixels, true);

    if(debugMessages)
        std::cout << closestPixelsMean << std::endl;

    if(drawImages)
        drawFloatImg(cv_ptr,closestPixelsMean);

    blobdetection::depth_point dp_msg;
    dp_msg.x = closestPixelsMean.x;
    dp_msg.y = closestPixelsMean.y;
    dp_msg.depth = closestPixelsMean.value;
    depth_point_publisher.publish(dp_msg);

    // normalize image, remove max values. Doing this normalisation results in the
    // smallest distances being found in places where there is no actual data
    // (i.e. on the borders of the image, in the regions where there is no data
    // due to camera positioning.)
    //ROS_INFO("Normalising image");
    //cv_ptr = normalize(cv_ptr);
    //if(1) return;
    //ROS_INFO("Naive detection");
    //KeyPoint kp = naiveDetection(cv_ptr, 500);
    //unnormalize depth.
//    kp.pt.y = (kp.pt.y * std_img) + mean_img;
    //  ROS_INFO_STREAM("x cor: " << kp.pt.x << " distance: " << kp.pt.y);

    // calculate kinematics and send twist to robot simulation node
    // geometry_msgs::Twist twist_msg;

    // twist_msg.linear.x = kp.pt.x;
    // twist_msg.linear.y = 0.0;
    // twist_msg.linear.z = kp.pt.y;

    // twist_msg.angular.x = 0.0;
    // twist_msg.angular.y = 0.0;
    // twist_msg.angular.z = 0.0;

    // blob_publisher.publish(twist_msg);
}


ros::NodeHandle ClosestPixelDetectorNode::nodeSetup(int argc, char* argv[]) {
    ros::init(argc, argv, "closestPixelDetector");
    ros::NodeHandle handle;

    if (!handle.getParam("/blobdetection/numClosestPixels", numClosestPixels)){
        ROS_ERROR("/blobdetection/numClosestPixels is not defined!");
        std::exit(1);
    }

    if (!handle.getParam("/blobdetection/drawImages", drawImages)){
        ROS_ERROR("/blobdetection/drawImages is not defined!");
        std::exit(1);
    }

    if (!handle.getParam("/blobdetection/debugMessages", debugMessages)){
        ROS_ERROR("/blobdetection/debugMessages is not defined!");
        std::exit(1);
    }


    t_depth = ros::Time::now();
    cv::namedWindow("BlobImage");
    cv::namedWindow("DepthImage");
    depth_subscriber = handle.subscribe("/camera/depth/image_raw", 1, &ClosestPixelDetectorNode::depthCallback, this);
    depth_point_publisher = handle.advertise<blobdetection::depth_point>("/vision/closest_blob", 1);
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

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv/highgui.h>

#include <iostream>

#include <limits>

using namespace cv;

class BlobDetectorNode {
public:

    ros::NodeHandle nh;
    ros::Subscriber depth_subscriber;
    ros::Publisher blob_publisher;

    BlobDetectorNode() {
        nh = ros::NodeHandle("blobdetector");

        t_depth = ros::Time::now();


        cv::namedWindow("windowname");


        depth_subscriber = nh.subscribe("/camera/depth/image_raw", 1, &BlobDetectorNode::depthCallback, this);
        blob_publisher = nh.advertise<geometry_msgs::Twist>("/vision/blobdetection", 1);
    }

    ~BlobDetectorNode(){

    }

    void depthCallback(const sensor_msgs::Image::ConstPtr &msg) {
        t_depth = ros::Time::now();
        img = msg;
    }
    
    cv_bridge::CvImagePtr convertImage() {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return cv_ptr;
        }
        /*Mat m(img->height,img->width,CV_32SC1);
        numsteps = img->step*img->height;

        for(int i = 0; i < numsteps; ++i) {

        }*/




        float min = std::numeric_limits<float>::max();
        float max = std::numeric_limits<float>::min();
        int count = 0;
        for (int i = 0; i < cv_ptr->image.rows; i++) {
            const float* Mi = cv_ptr->image.ptr<float>(i);
            for(int j = 0; j < cv_ptr->image.cols; j++){
             //if (j = 0){
             //    ROS_INFO_STREAM("" << Mi[j]);
             //}
             //   std::cout << Mi[j] << std::endl;
             if (Mi[j] > 1e10){

                 count++;
                  //cv_ptr->image.at<double>(i, j) = 255.0;
                  continue;
            }

            if (Mi[j] < min)
                min = Mi[j];
            if (Mi[j] > max)
                max = Mi[j];
            }

        }

        maxval = max;

        //ROS_INFO_STREAM("Min: " << min << ", Max: " << max << ", Count: " << count);

        for (int i = 0; i < cv_ptr->image.rows; ++i)
        {
            for(int j = 0; j < cv_ptr->image.cols; ++j){
                //if(cv_ptr->image.at<double>(i,j) )
                cv_ptr->image.at<float>(i,j) =  (cv_ptr->image.at<float>(i,j) / max)*255;
            }

        }


        Mat m(img->height,img->width,CV_8UC1);
        cv_ptr->image.convertTo(m,CV_8UC1);

        cv_ptr->image = m;



        return cv_ptr;
    }
    
    cv_bridge::CvImagePtr normalize(cv_bridge::CvImagePtr cv_ptr) {
        return cv_ptr; //TODO
    }
    
    /*Detect blobs from the stored image*/
    vector<KeyPoint> detectBlobs(cv_bridge::CvImagePtr cv_ptr) {
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
    
    KeyPoint getClosestBlob(vector<KeyPoint> blobs, cv_bridge::CvImagePtr cv_ptr) {
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
    

    void update() {
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

private:

    ros::Time t_depth;
    sensor_msgs::Image::ConstPtr img;
    float maxval;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "blobDetector");

    BlobDetectorNode bdn;

    // Control @ 10 Hz
    double control_frequency = 10.0;

    ros::Rate loop_rate(control_frequency);
    for(int i = 0; i < 10; ++i)
        loop_rate.sleep();


    while(bdn.nh.ok()) {
        bdn.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace cv;

class BlobDetectorNode {
public:

    ros::NodeHandle nh;
    ros::Subscriber depth_subscriber;
    ros::Publisher blob_publisher;

    BlobDetectorNode() {
        nh = ros::NodeHandle("blobdetector");

        t_depth = ros::Time::now();

        depth_subscriber = nh.subscribe("/image_converter/output_video", 1, &BlobDetectorNode::depthCallback, this);
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
        return cv_ptr;
    }
    
    cv_bridge::CvImagePtr normalize(cv_bridge::CvImagePtr cv_ptr) {
        return cv_ptr; //TODO
    }
    
    /*Detect blobs from the stored image*/
    vector<KeyPoint> detectBlobs(cv_bridge::CvImagePtr cv_ptr) {
        vector<KeyPoint> kps;
        return kps; //TODO
    }
    
    KeyPoint getClosestBlob(vector<KeyPoint> blobs, cv_bridge::CvImagePtr cv_ptr) {
        KeyPoint kp;
        return kp; //TODO
    }
    

    void update() {
        // if more than 2 seconds have passed and no messages have been received,
        // stop sending
        if((ros::Time::now()-t_depth).toSec()>2.0) {
            return;
        }
        
        //convert image to openCV image
        cv_bridge::CvImagePtr cv_ptr = convertImage();
        //normalize image, remove max values
        cv_ptr = normalize(cv_ptr);
        //detect blobs in image
        vector<KeyPoint> points = detectBlobs(cv_ptr);
        //pick the closest blob
        KeyPoint kp = getClosestBlob(points,cv_ptr);


        // calculate kinematics and send twist to robot simulation node
        geometry_msgs::Twist twist_msg;

        twist_msg.linear.x = kp.pt.x;
        twist_msg.linear.y = kp.pt.y;
        twist_msg.linear.z = 0.0;

        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = 0.0;

        blob_publisher.publish(twist_msg);

    }

private:

    ros::Time t_depth;
    sensor_msgs::Image::ConstPtr img;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "blobDetector");

    BlobDetectorNode bdn;

    // Control @ 10 Hz
    double control_frequency = 10.0;

    ros::Rate loop_rate(control_frequency);

    while(bdn.nh.ok()) {
        bdn.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

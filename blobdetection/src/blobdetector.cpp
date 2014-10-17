#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>


class BlobDetectorNode {
public:

    ros::NodeHandle nh;
    ros::Subscriber depth_subscriber;
    ros::Publisher blob_publisher;

    BlobDetectorNode() {
        nh = ros::NodeHandle("blobdetector");

        pwm_ = std::vector<int>(2, 0);
        t_pwm_ = ros::Time::now();

        depth_subscriber = nh.subscribe("/camera/depth/image_rect", 1, &BlobDetectorNode::depthCallback, this);
        blob_publisher = nh.advertise<geometry_msgs::Twist>("/vision/blobdetection", 1);
    }

    ~BlobDetectorNode(){
        
    }

    void depthCallback(const sensor_msgs::Image::ConstPtr &msg) {
        t_depth = ros::Time::now();
        img = msg;
    }
    
    /*Detect blobs from the stored image*/
    vector<cv::KeyPoint> detectBlobs() {
        
    }
    
    

    void update() {
        // if more than 2 seconds have passed and no messages have been received,
        // stop sending
        if((ros::Time::now()-t_depth).toSec()>2.0) {
            return;
        }


        // calculate kinematics and send twist to robot simulation node
        geometry_msgs::Twist twist_msg;

        double linear_vel = (wheel_angular_velocities[1] + wheel_angular_velocities[0])*0.5*wheel_radius_;
        double angular_vel = (wheel_angular_velocities[1] - wheel_angular_velocities[0])*wheel_radius_/base_;

        twist_msg.linear.x = linear_vel;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;

        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = angular_vel;

        blob_publisher.publish(twist_msg);

    }

private:

    ros::Time t_depth;
    
    const sensor_msgs::Image::ConstPtr &img;

    double wheel_radius_;
    double base_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "blobdetector");

    BlobDetectorNode bdn;

    // Control @ 10 Hz
    double control_frequency = 10.0;

    ros::Rate loop_rate(control_frequency);

    while(kobuki_motors_node.n_.ok()) {
        kobuki_motors_node.updateMotors();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

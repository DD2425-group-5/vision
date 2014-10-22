#include "purpleClassifierNode.hpp"


void PurpleClassifierNode::rgbCallback(const sensor_msgs::Image::ConstPtr &msg) {
    t_rgb = ros::Time::now();
    img = msg;
}

cv_bridge::CvImagePtr PurpleClassifierNode::convertImage() {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return cv_ptr;
    }
    return cv_ptr;
}

/**
When reading uint8 values into a char, and then casting into int,
the values will be between -128 and 127. But its not linear.
Values 0-127 correspond to actual value 0-127. But then it continues
to -128 and goes to -1. Basically:

-128 -> 128
-127 -> 129
...
-1 -> 255

So for negative values, we want to add 256 to it.
*/
int PurpleClassifierNode::toRgbInt(int i)  {
    if(i >= 0)
        return i;

    return i + 256;
}


cv_bridge::CvImagePtr PurpleClassifierNode::discretizeImage(cv_bridge::CvImagePtr cv_ptr) {
    purple_points.empty();

    //iterate through the image, check all pixels if they are purple, and put the purple ones in
    //the vector.
    for (int i = 0; i < cv_ptr->image.rows; ++i) {
    for (int j = 0; j < cv_ptr->image.cols; ++j) {
        cv::Vec3b& tmp_vec = cv_ptr->image.at<cv::Vec3b>(i,j);
        if(isPurple(tmp_vec[0],tmp_vec[1],tmp_vec[2])) {
            purple_points.push_back(cv::Point_<int>(j,i));
            tmp_vec = purple_rgb;
        } else {
            purple_points.push_back(cv::Point_<int>(j,i));
            tmp_vec = non_purple_rgb;
        }
    }
    }

    return cv_ptr;
}

/**
Check if a pixel color is purple. Input should be ints between 0 and 255.
The parameters are set to doubles because then the calculations will be more accurate.
*/
bool PurpleClassifierNode::isPurple(double r, double g, double b) {

    if(g <= 1e-6 || b <= 1e-6)
        return false;

    double rg_ratio = r/g;
    double rb_ratio = r/b;


    if(rg_ratio < 2 || rg_ratio > 3 || rb_ratio < 1 || rb_ratio > 2)
        return false;


    return true;
}

void PurpleClassifierNode::update() {
    if((ros::Time::now()-t_rgb).toSec()>1.0) {
        return;
    }

    //convert image to openCV image
    cv_bridge::CvImagePtr cv_ptr = convertImage();

    cv_ptr = discretizeImage(cv_ptr);

    cv::imshow("purpleImage", cv_ptr->image);
    cv::waitKey(3);

}

ros::NodeHandle PurpleClassifierNode::nodeSetup(int argc, char* argv[]) {
    ros::init(argc, argv, "PurpleClassifier");
    ros::NodeHandle handle;

    purple_rgb.val[0] = 255;
    purple_rgb.val[1] = 102;
    purple_rgb.val[2] = 168;

    non_purple_rgb[0] = 0;
    non_purple_rgb[1] = 0;
    non_purple_rgb[2] = 0;

    cv::namedWindow("purpleImage");

    t_rgb = ros::Time::now();
    rgb_subscriber = handle.subscribe("/camera/rgb/image_rect_color", 1, &PurpleClassifierNode::rgbCallback, this);
    classifier_publisher = handle.advertise<sensor_msgs::Image>("/vision/purple_classifier", 1);
    return handle;
}


void PurpleClassifierNode::runNode(ros::NodeHandle handle) {
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

PurpleClassifierNode::PurpleClassifierNode(int argc, char* argv[]) {
    ros::NodeHandle handle = nodeSetup(argc, argv);
    runNode(handle);
}

int main(int argc, char* argv[])
{
    PurpleClassifierNode pcn(argc, argv);
}

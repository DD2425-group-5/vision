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


void PurpleClassifierNode::discriminateImage(cv_bridge::CvImagePtr cv_ptr) {
    //purple_points.empty();

    //cv::Ptr p();


    //cv::Mat mp = cv::Mat::zeros(im.rows, im.cols, CV_32F);

    //iterate through the image, check all pixels if they are purple, and put the purple ones in
    //the vector.
    for (int i = 0; i < cv_ptr->image.rows; ++i) {
    for (int j = 0; j < cv_ptr->image.cols; ++j) {
        const cv::Vec3b& tmp_vec = cv_ptr->image.at<cv::Vec3b>(i,j);
        disc_image.at<float>(i,j) = discriminant(tmp_vec.val[0],tmp_vec.val[1],tmp_vec.val[2],0.5f,purple_model);
    }
    }
    return;
}

float PurpleClassifierNode::gauss(float x, float mu, float sigma) {
    return std::pow(x - mu, 2)/(2*std::pow(sigma,2));
}


/**
 * class_prob = log(prior) - sum(log(sigmas)) - sum(((x-mu)^2)/2sigma^2)
 */
float PurpleClassifierNode::discriminant(float r, float g, float b, float prior, const ModelParams &model) {
    float sigmasum = std::log(model.std_b) + std::log(model.std_g) + std::log(model.std_r);
    float musum = gauss(b, model.mu_b, model.std_b)
    + gauss(g, model.mu_g, model.std_g)
    + gauss(r, model.mu_r, model.std_r);

    float res = std::log(prior) - sigmasum - musum;

    //ROS_INFO_STREAM("musum " << musum);
    return res;
}

void PurpleClassifierNode::update() {
    if((ros::Time::now()-t_rgb).toSec()>1.0) {
        return;
    }

    //convert image to openCV image
    cv_bridge::CvImagePtr cv_ptr = convertImage();

    discriminateImage(cv_ptr);

    double max, min;
    cv::minMaxIdx(disc_image, &min, &max);
    float minm = min - min;
    float maxm = max - min;

    ROS_INFO_STREAM("min: " << min << " max: " << max);

    cv::Mat gray = cv::Mat::zeros(disc_image.rows, disc_image.cols, CV_32F);
    for (int row = 0; row < gray.rows; row++) {
        for (int col = 0; col < gray.cols; col++) {
        gray.at<float>(row, col) = (((disc_image.at<float>(row, col) - min)/maxm));
        }
    }

    //minMaxIdx(gray, &min, &max);

    cv::imshow("class", gray);

    //cv::imshow("discImage", disc_image);
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

    /*object_models:
      purple_cross:
        mu_r: 84.98354533597
        mu_g: 60.6422152725813
        mu_b: 84.1680430098564
        std_r: 17.3562073603737
        std_g: 21.104604511573
        std_b: 18.7768787379336
      purple_cross2:
        mu_r: 102.367348301744
        mu_g: 85.8447735151
        mu_b: 125.755065593005
        std_r: 25.5595663732303
        std_g: 27.7259334260372
        std_b: 25.4226051646474
      model_names: [purple_cross, purple_cross2]*/

    purple_model = ModelParams("purple_cross",84.98,60.64,84.17,17.36,21.10,18.77);
    disc_image = cv::Mat::zeros(480,640,CV_32F);

    cv::namedWindow("purpleImage");
    cv::namedWindow("class", CV_WINDOW_AUTOSIZE);

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

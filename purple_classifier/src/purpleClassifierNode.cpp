#include "purpleClassifierNode.hpp"
#define PI 3.14159265359

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
    //


    for (int i = 0; i < cv_ptr->image.rows; ++i) {
    for (int j = 0; j < cv_ptr->image.cols; ++j) {
        const cv::Vec3b& pixel = cv_ptr->image.at<cv::Vec3b>(i,j);
        double r = pixel.val[2];
        double g = pixel.val[1];
        double b = pixel.val[0];
        double intensity = r+b+g;
        r = r/intensity;
        g = g/intensity;


        disc_image.at<float>(i,j) = discriminant(r,g);
    }
    }
    return;
}

float PurpleClassifierNode::gauss(float x, float mu, float sigma) {
    return std::pow(x - mu, 2)/(2*std::pow(sigma,2));
}


/**
 * http://en.wikipedia.org/wiki/Multivariate_normal_distribution
 */
float PurpleClassifierNode::discriminant(double r, double g) {
    double rn = r-purple_model.mu[0];
    double gn = g-purple_model.mu[1];

    double res = constant*std::exp(-0.5 * (rn*rn*sigma_inv[0][0] + 2*rn*gn*sigma_inv[0][1] + gn*gn*sigma_inv[1][1]));


    //ROS_INFO_STREAM("musum " << musum);
    return res;
}

void PurpleClassifierNode::calc_inv_sigma() {
    //only works for dimension 2;
    //http://mathworld.wolfram.com/MatrixInverse.html
    sigma_inv = std::vector<std::vector<double> >(2);
    sigma_inv[0] = std::vector<double>(2);
    sigma_inv[1] = std::vector<double>(2);
    sigma_inv[0][0] = purple_model.sigma[1][1] / sigma_det;
    sigma_inv[0][1] = -(purple_model.sigma[0][1]) / sigma_det;
    sigma_inv[1][0] = -(purple_model.sigma[1][0]) / sigma_det;
    sigma_inv[1][1] = purple_model.sigma[0][0] / sigma_det;
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

    /*
    models:
      purpleModel:
        mu:
          - 0.341743854434246
          - 0.241070271079385
        sigma:
          - 0.00214422574915891
          - 0.000934942249428683
          - 0.000934942249428683
          - 0.00159702365225887*/

    purple_model = color_model_vardim<double>(2);
    purple_model.mu[0] = 0.341743854434246;
    purple_model.mu[1] = 0.241070271079385;
    purple_model.sigma[0][0] = 0.00214422574915891;
    purple_model.sigma[0][1] = 0.000934942249428683;
    purple_model.sigma[1][0] = 0.000934942249428683;
    purple_model.sigma[1][1] = 0.00159702365225887;

    //http://en.wikipedia.org/wiki/Multivariate_normal_distribution
    sigma_det = (purple_model.sigma[0][0]*purple_model.sigma[1][1]) -
                (purple_model.sigma[0][1]*purple_model.sigma[1][0]);
    constant = 1.0d / std::sqrt(std::pow(2*PI,2)*sigma_det);
    calc_inv_sigma();

            //ModelParams("purple_cross",84.98,60.64,84.17,17.36,21.10,18.77);
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

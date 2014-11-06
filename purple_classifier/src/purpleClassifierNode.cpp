#include "purpleClassifierNode.hpp"

using namespace VisionModels;

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


void PurpleClassifierNode::discriminateImage(cv_bridge::CvImagePtr cv_ptr, std::vector<cv::Vec2i> &purple_points) {
    //cv::Ptr p();


    //cv::Mat mp = cv::Mat::zeros(im.rows, im.cols, CV_32F);

    //iterate through the image, check all pixels if they are purple, and put the purple ones in
    //the vector.
    //

    //cv::Scalar sc = cv::mean(cv_ptr->image);
    //r_wp = sc[2]/(sc[0]+sc[1]+sc[2]);
    //g_wp = sc[1]/(sc[0]+sc[1]+sc[2]);

    for (int i = 0; i < cv_ptr->image.rows; ++i) {
    for (int j = 0; j < cv_ptr->image.cols; ++j) {
        const cv::Vec3b& pixel = cv_ptr->image.at<cv::Vec3b>(i,j);
        double r = pixel.val[2];
        double g = pixel.val[1];
        double b = pixel.val[0];
        if(!process_pixel(r,g,b)) {
            disc_image.at<float>(i,j) = 0;
            continue;
        }


        double intensity = r+b+g;
        r = r/intensity;
        g = g/intensity;

        /*
        double l2 = std::pow(r,2)+std::pow(g,2);
        double m = c_r*r + c_g*g;
        double m2 = std::pow(m,2);
        //if(i%10 == 0 && j%10 == 0)
        //ROS_INFO_STREAM("l2: " << l2 << " m: " << m << " m2:" << m2);

        //ROS_INFO_STREAM("s_min2: " << s_min2 << " s_max2:" << s_max2 << " l2: " << l2);
        //ROS_INFO_STREAM("w2: " << w2 << " m: " << m << " m2:" << std::pow(m,2) << " l2: " << l2);

        if(!(s_min2 < l2))
            ROS_INFO("S_MIN2 < l2 FAILED");
        if(!(s_max2 > l2)) {
            ROS_INFO("S_MAX2 > l2 FAILED");
            ROS_INFO_STREAM("smax2: " << s_max2 << " l2: " << l2);
        }

        if(!(m2 > w2*l2)) {
            ROS_INFO("M2 > w2*l2 FAILED");
            ROS_INFO_STREAM("w2: " << w2 << " m2: " << m2 << " l2: " << l2);
        }

        if(
                s_min2 < l2 &&
                l2 < s_max2 &&
                //m > 0 &&
                m2 > w2*l2 ) {

            ROS_INFO("INSIDE");
            purple_points.push_back(cv::Vec2i(i,j));
        }
*/


        double rho = std::atan((r-r_wp)/(g-g_wp));

        //double s_diff = purple_model.sigma[1][1]*100;

        double rho_diff = purple_model.sigma[0][0]*2;

        if( rho > purple_model.mu[0] - rho_diff &&
            rho < purple_model.mu[0] + rho_diff) {
                double s = std::sqrt(std::pow(r-r_wp,2)+std::pow(g-g_wp,2));
                if(s > s_min && s < s_max) {
                    purple_points.push_back(cv::Vec2i(i,j));
                }


        }


            //disc_image.at<float>(i,j) = 1.0;




        //else
        //    disc_image.at<float>(i,j) = 0;

        //if(rho < )

        //disc_image.at<float>(i,j) = discriminant(rho,s);
    }
    }
    return;
}

bool PurpleClassifierNode::process_pixel(double r, double g, double b) {
    if(r+b+g > 700)
        return false;

    if(r+b+g < 100)
        return false;

    //if(r > 220 && g > 220 && b > 220)
    //    return false;




    return true;
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

    //ROS_INFO("DISCRIMINATE_BEGIN");
    std::vector<cv::Vec2i> purple_points;
    discriminateImage(cv_ptr,purple_points);
    //ROS_INFO("DISCRIMINATE_END");
    //double max, min;
    //cv::minMaxIdx(disc_image, &min, &max);
    //float minm = min - min;
    //float maxm = max - min;

    std::vector<clustering::Cluster> clusts;
    clustering::cluster_img(purple_points,80,clusts);
    //ROS_INFO("CLUSTERING");
    clustering::Cluster cl = clustering::get_biggest_cluster(clusts);
    //ROS_INFO("BIGGEST_CLUSTER");

    if(cl.size() > 500)
        ROS_INFO("OBJECT_DETECTED");
    else
        ROS_INFO("NOPE");

    /*
    cv::Mat white = cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_32F);
    if(cl.points.size() > 100)
    for(int i = 0; i < cl.points.size(); ++i) {
        white.at<float>(cl.points[i][0],cl.points[i][1]) = 0.99;
    }
    ROS_INFO("SET_WHITE");
*/
    //ROS_INFO_STREAM("min: " << min << " max: " << max);
/*
    cv::Mat gray = cv::Mat::zeros(disc_image.rows, disc_image.cols, CV_32F);
    for (int row = 0; row < gray.rows; row++) {
        for (int col = 0; col < gray.cols; col++) {
            gray.at<float>(row, col) = (((disc_image.at<float>(row, col) - min)/maxm));
        }
    }
*/
    //minMaxIdx(gray, &min, &max);

    //cv::imshow("class", white);

    //cv::imshow("discImage", disc_image);
    //cv::waitKey(3);

}

void PurpleClassifierNode::readModel(ros::NodeHandle& n) {
    std::string modelName = "purpleModel";
    std::vector<double> sig(4);
    std::vector<double> mu(2);
    ROSUtil::getParam(n, "/models/" + modelName + "/sigma", sig);
    ROSUtil::getParam(n,"/models/" + modelName + "/mu", mu);

    ROS_INFO_STREAM("mu: " << mu[0] << " " << mu[1]);
    ROS_INFO_STREAM("sig: " << sig[0] << " " << sig[1] << " " << sig[2] << " " << sig[3]);

    purple_model.mu[0] = mu[0];
    purple_model.mu[1] = mu[1];
    purple_model.sigma[0][0] = sig[0];
    purple_model.sigma[0][1] = sig[1];
    purple_model.sigma[1][0] = sig[2];
    purple_model.sigma[1][1] = sig[3];
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

    r_wp = ((double) 1.0)/((double) 3.0);
    g_wp = ((double) 1.0)/((double) 3.0);



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
    readModel(handle);

    //if(purple_model.mu[0] < 0)
    //    purple_model.mu[0] += (2*M_PI);

    s_min = purple_model.mu[1]-purple_model.sigma[1][1]*100;
    s_max = purple_model.mu[1]+purple_model.sigma[1][1]*100;
    rho_diff = purple_model.sigma[0][0]*2;

    s_min2 = std::pow(s_min,2);
    s_max2 = std::pow(s_max,2);
    //c_r = std::sin((M_PI*purple_model.mu[0])/180);
    //c_g = std::cos((M_PI*purple_model.mu[0])/180);
    //w2 = std::pow(std::cos((M_PI*rho_diff)/180),2);



    c_r = std::sin(purple_model.mu[0]);
    //if(c_r < 0)
    //    c_r += (2*M_PI);

    c_g = std::cos(purple_model.mu[0]);
    //if(c_g < 0)
    //    c_g += (2*M_PI);
    w2 = std::pow(std::cos(rho_diff),2);


    //c_r = std::sin(2*M_PI*purple_model.mu[0] / (360));
    //c_g = std::cos(2*M_PI*purple_model.mu[0] / (360));
    ROS_INFO("PARAMETERS:");
    ROS_INFO_STREAM("rho: " << purple_model.mu[0] << ", rho_diff: "  << rho_diff);
    ROS_INFO_STREAM("s_min: " << s_min << ", s_max: "  << s_max);
    ROS_INFO_STREAM("s_min2: " << s_min2 << ", s_max2: "  << s_max2);
    ROS_INFO_STREAM("c_r: " << c_r << ", c_g: "  << c_g);
    ROS_INFO_STREAM("w2: " << w2);





    //http://en.wikipedia.org/wiki/Multivariate_normal_distribution
    sigma_det = (purple_model.sigma[0][0]*purple_model.sigma[1][1]) -
                (purple_model.sigma[0][1]*purple_model.sigma[1][0]);
    constant = 1.0d / std::sqrt(std::pow(2*M_PI,2)*sigma_det);
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

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


void PurpleClassifierNode::discriminateImage(cv_bridge::CvImagePtr cv_ptr, cv::Mat &disc_image) {
    //purple_points.clear();

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



        //else
        //    disc_image.at<float>(i,j) = 0;

        //if(rho < )

        disc_image.at<float>(i,j) = discriminant(r,g);
    }
    }
    return;
}

bool PurpleClassifierNode::process_pixel(double r, double g, double b) {
    if(r+b+g > rgb_sum_thresh_max)
        return false;

    if(r+b+g < rgb_sum_thresh_min)
        return false;

    //if(r > 220 && g > 220 && b > 220)
    //    return false;




    return true;
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

void PurpleClassifierNode::get_pixel_vector(cv::Mat& src, float thresh,std::vector<cv::Vec2i>& out) {

    int largerthan1 = 0;
    for (int i = 0; i < src.rows; ++i) {
    for (int j = 0; j < src.cols; ++j) {
        //ROS_INFO_STREAM("SRC AT: " << src.at<float>(i,j));
        //if(src.at<float>(i,j) > 1.0)
        //    largerthan1 += 1;
        if(src.at<float>(i,j) > thresh) {
            out.push_back(cv::Vec2i(i,j));
        }


    }
    }

    //ROS_INFO_STREAM("pixels larger than 1: " << largerthan1);
}

bool PurpleClassifierNode::is_object(
        const std::vector<float>& row_sums, const std::vector<float>& col_sums,
        float thresh_row,float thresh_col,int lines_col, int lines_row) {

    return is_object_help(row_sums,thresh_row,lines_row) &&
           is_object_help(col_sums,thresh_col,lines_col);

}

bool PurpleClassifierNode::is_object_help(const std::vector<float>& vec, float thresh, int lines) {
    int yes_in_a_row = 0;
    for(int i = 0; i < vec.size(); ++i) {
        if(vec[i] > thresh) {
            ++yes_in_a_row;
            if(yes_in_a_row >= lines) {
                return true;
            }
        } else {
            yes_in_a_row = 0;
        }
    }
    return false;
}



void PurpleClassifierNode::update() {
    if((ros::Time::now()-t_rgb).toSec()>1.0) {
        return;
    }

    //convert image to openCV image
    cv_bridge::CvImagePtr cv_ptr = convertImage();


    //ROS_INFO("DISCRIMINATE_BEGIN");
    cv::Mat disc_image = cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_32F);
    discriminateImage(cv_ptr,disc_image);
    //ROS_INFO("DISCRIMINATE_END");

    cv::Mat blurred = cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_32F);
    cv::blur(disc_image,blurred,cv::Size(blur_size,blur_size));
    //ROS_INFO("BLUR END");

    //std::vector<cv::Vec2i> purple_points;
    //get_pixel_vector(blurred,1,purple_points);
    //ROS_INFO_STREAM("purple points size: " << purple_points.size());



    double maxb, minb,min,max;
    cv::minMaxIdx(disc_image, &min, &max);
    //cv::Scalar std,mean;
    //cv::meanStdDev(disc_image,mean,std);
    //ROS_INFO_STREAM("MEAN: " << mean << " STD: " << std);
    cv::minMaxIdx(blurred, &minb, &maxb);
    float maxm = max - min;
    float maxmb = maxb-minb;

    std::vector<float> row_sums(blurred.rows);
    std::vector<float> col_sums(blurred.cols);
    clustering::rows_sum(blurred,row_sums);
    clustering::cols_sum(blurred,col_sums);

    //int lines_col = 60;
    //int lines_row = 60;
    //float thresh_col = 1000;
    //float thresh_row = 1000;

    bool found = is_object(row_sums,col_sums,thresh_row,thresh_col, lines_col, lines_row);

    if(found)
        ROS_INFO("OBJECT DETECTED!");
    else
        ROS_INFO("NOPE");


    /*
    std::vector<clustering::Cluster> clusts;
    clustering::cluster_img_mat(blurred,2,80,clusts);
    ROS_INFO("CLUSTERING");
    clustering::Cluster cl = clustering::get_biggest_cluster(clusts);
    ROS_INFO("BIGGEST_CLUSTER");
    cv::Mat white = cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_32F);

    if(cl.points.size() > 100)
    for(int i = 0; i < cl.points.size(); ++i) {
        white.at<float>(cl.points[i][0],cl.points[i][1]) = 1.0;
    }
    ROS_INFO("SET_WHITE");
        */
    /*cv::Mat white = cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_32F);

    for(int i = 0; i < purple_points.size(); ++i) {
        white.at<float>(purple_points[i][0],purple_points[i][1]) = 1.0;
    }*/
    //ROS_INFO_STREAM("CLUSTER SIZE: " << cl.size());
    //ROS_INFO_STREAM("min: " << min << " max: " << max);
    //ROS_INFO_STREAM("minblur: " << minb << " maxblur: " << maxb);
    cv::Mat gray = cv::Mat::zeros(disc_image.rows, disc_image.cols, CV_32F);
    cv::Mat gray_blur = cv::Mat::zeros(disc_image.rows, disc_image.cols, CV_32F);
    for (int row = 0; row < gray.rows; row++) {
        for (int col = 0; col < gray.cols; col++) {
            gray.at<float>(row, col) = (((disc_image.at<float>(row, col) - min)/maxm));
            gray_blur.at<float>(row, col) = (((blurred.at<float>(row, col) - minb)/maxmb));
        }
    }
    //ROS_INFO("IMAGE WRITE");

    //minMaxIdx(gray, &min, &max);

    cv::imshow("blurred", gray_blur);
    cv::imshow("noblur", gray);
    //cv::imshow("clustered", white);

    //cv::imshow("discImage", disc_image);
    cv::waitKey(3);

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

void PurpleClassifierNode::readParams(ros::NodeHandle& n) {
    ROSUtil::getParam(n, "/purple_classifier_params/rgb_sum_thresh_max", rgb_sum_thresh_max);
    ROSUtil::getParam(n, "/purple_classifier_params/rgb_sum_thresh_min", rgb_sum_thresh_min);
    ROSUtil::getParam(n, "/purple_classifier_params/blur_kernel_size", blur_size);
    ROSUtil::getParam(n, "/purple_classifier_params/lines_col", lines_col);
    ROSUtil::getParam(n, "/purple_classifier_params/lines_row", lines_row);
    ROSUtil::getParam(n, "/purple_classifier_params/thresh_col", thresh_col);
    ROSUtil::getParam(n, "/purple_classifier_params/thresh_row", thresh_row);

    ROS_INFO_STREAM("Parameters read. rgb_sum_thresh max and min: " << rgb_sum_thresh_max
                    << " " << rgb_sum_thresh_min);
    ROS_INFO_STREAM("Blur kernel size set to (" << blur_size << "," << blur_size << ")");
    ROS_INFO_STREAM("lines_col: " << lines_col << " lines_row: " << lines_row
                    << " thresh_col: " << thresh_col << " thresh_row: " << thresh_row);
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
    readModel(handle);
    readParams(handle);


    //http://en.wikipedia.org/wiki/Multivariate_normal_distribution
    sigma_det = (purple_model.sigma[0][0]*purple_model.sigma[1][1]) -
                (purple_model.sigma[0][1]*purple_model.sigma[1][0]);
    constant = 1.0d / std::sqrt(std::pow(2*M_PI,2)*sigma_det);
    calc_inv_sigma();

            //ModelParams("purple_cross",84.98,60.64,84.17,17.36,21.10,18.77);
    //disc_image = cv::Mat::zeros(480,640,CV_32F);

    //cv::namedWindow("purpleImage");
    cv::namedWindow("noblur", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("blurred", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("clustered", CV_WINDOW_AUTOSIZE);

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

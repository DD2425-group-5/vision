#include "colordetectionnode.hpp"
void ColorDetectionNode::rgbCallback(const sensor_msgs::Image::ConstPtr &msg) {
    t_rgb = ros::Time::now();
    camera_img_raw; = msg;
}

cv_bridge::CvImagePtr ColorDetectionNode::convertImage() {
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
Converts the src rgb image rg_chromasity output image.

NOTE:
assumes output is already initialized to size = src.rows,src.cols , and the
type should be CV_32FC2. src is assumed to be of type rgb8.
*/
void ColorDetectionNode::rgb2rgChromasity(const cv::Mat &src, cv::Mat &output) {
    for(int row = 0; row < src.rows; ++row) {
        for(int col = 0; col < src.cols; ++col) {
            const cv::Vec3b& pixel = cv_ptr->image.at<cv::Vec3b>(i,j);
            double r = pixel.val[0];
            double g = pixel.val[1];
            double b = pixel.val[2];

            double intensity = r+b+g;
            r = r/intensity;
            g = g/intensity;

            output.at<cv::Vec2f>(row,col) = cv::Vec2f(r,g);
        }
    }
}

/**
Read parameters for model and algorithm parameters for color color_model_name.
Every value is put in the given struct &cap. Assumed that it is just initialized with default
constructor.
  */
void ColorDetectionNode::readModel(ros::NodeHandle n, std::string color_model_name,color_alg_params &cap) {
    //read color model
    VisionModels::color_model_vardim<double> color_model;
    std::vector<double> sig(4);
    std::vector<double> mu(2);
    ROSUtil::getParam(n, "/models/" + color_model_name + "/sigma", sig);
    ROSUtil::getParam(n,"/models/" + color_model_name + "/mu", mu);
    color_model.mu[0] = mu[0];
    color_model.mu[1] = mu[1];
    color_model.sigma[0][0] = sig[0];
    color_model.sigma[0][1] = sig[1];
    color_model.sigma[1][0] = sig[2];
    color_model.sigma[1][1] = sig[3];

    cap.color_model = color_model;

    ROS_INFO("====================================================");
    ROS_INFO_STREAM("Parameters read for model " << color_model_name);
    ROS_INFO_STREAM("mu: " << mu[0] << " " << mu[1]);
    ROS_INFO_STREAM("sig: " << sig[0] << " " << sig[1] << " " << sig[2] << " " << sig[3]);


    //read color algorithm parameters
    ROSUtil::getParam(n,"/models/" + color_model_name + "/blur_kernel_size", cap.blur_size);
    ROSUtil::getParam(n,"/models/" + color_model_name + "/lines_col", cap.lines_col);
    ROSUtil::getParam(n,"/models/" + color_model_name + "/lines_row", cap.lines_row);
    ROSUtil::getParam(n,"/models/" + color_model_name + "/thresh_row", cap.thresh_row);
    ROSUtil::getParam(n,"/models/" + color_model_name + "/thresh_col", cap.thresh_col);

    ROS_INFO_STREAM("Blur kernel size set to (" << blur_size << "," << blur_size << ")");
    ROS_INFO_STREAM("lines_col: " << lines_col << " lines_row: " << lines_row
                    << " thresh_col: " << thresh_col << " thresh_row: " << thresh_row);


}


ColorDetectionNode::update() {
    if((ros::Time::now()-t_rgb).toSec()>1.0) {
        return;
    }

    //convert image to openCV image
    cv_bridge::CvImagePtr cv_ptr = convertImage();

    //convert to rg-chromasity image.
    cv::Mat rg_chrom_img = cv::Mat::zeros(cv_ptr->image.rows,cv_ptr->image.cols, CV_32FC2);



}

ColorDetectionNode::runNode(ros::NodeHandle handle) {
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

ColorDetectionNode::nodeSetup(int argc, char *argv[]) {
    ros::init(argc, argv, "PurpleClassifier");
    ros::NodeHandle handle;

    //read and store models
    //TODO: for now only reads purple
    color_alg_params cap;
    readModel(handle,"purple",cap);
    models.push_back(cap);


    t_rgb = ros::Time::now();
    rgb_subscriber = handle.subscribe("/camera/rgb/image_rect_color", 1, &PurpleClassifierNode::rgbCallback, this);
    classifier_publisher = handle.advertise<sensor_msgs::Image>("/vision/color_classifier", 1);
    return handle;
}

ColorDetectionNode::ColorDetectionNode(int argc, char *argv[]) {
    ros::NodeHandle handle = nodeSetup(argc, argv);
    runNode(handle);
}

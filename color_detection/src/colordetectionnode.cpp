#include "colordetectionnode.hpp"

const int ColorDetectionNode::numColors = 6;
const std::string ColorDetectionNode::colors[] = {"blue","green","red","yellow","orange","purple"};

void ColorDetectionNode::rgbCallback(const sensor_msgs::Image::ConstPtr &msg) {
    t_rgb = ros::Time::now();
    camera_img_raw = msg;
}

void ColorDetectionNode::depthCallback(const sensor_msgs::Image::ConstPtr &msg) {
    camera_depth_raw = msg;
}

cv_bridge::CvImagePtr ColorDetectionNode::convertImage(sensor_msgs::Image::ConstPtr img, const std::string& format) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img, format);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return cv_ptr;
    }
    return cv_ptr;
}

/**
Scales all values to between 0 and 1. Used for drawing a float image.
*/
void ColorDetectionNode::scaleImage(const cv::Mat& img, cv::Mat& output) {
    double min,max;
    output = cv::Mat::zeros(img.rows, img.cols, CV_32F);
    cv::minMaxIdx(img, &min, &max);

    //float maxm = max - min;
    for (int row = 0; row < output.rows; row++) {
        for (int col = 0; col < output.cols; col++) {
            output.at<float>(row, col) = (img.at<float>(row, col) - min)/max;
        }
    }
}

/**
Converts the src rgb image rg_chromasity output image.

NOTE:
reinitializes output.
*/
void ColorDetectionNode::rgb2rgChromasity(const cv::Mat &src, cv::Mat &output) {
    output = cv::Mat(src.rows, src.cols, CV_32FC2);

    for(int row = 0; row < src.rows; ++row) {
        for(int col = 0; col < src.cols; ++col) {
            const cv::Vec3b& pixel = src.at<cv::Vec3b>(row,col);
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
Discriminate against some images. We don't take kindly to their types.

Calculates the multivariate gaussian of every pixel rg-chromasity values.
See:
en.wikipedia.org/wiki/Multivariate_normal_distribution

some effectivizations:
This is the original calculation:

double res = model.gauss_constant *
   std::exp(-0.5 * (rn*rn*model.sigma_inv[0][0] +
                    2*rn*gn*model.sigma_inv[0][1] +
                    gn*gn*model.sigma_inv[1][1]));

Then the threshold was set to model.gauss_thresh
First, some calculations can be moved out, specifically the -0.5
multiplied with the constants within.
This gives the calculation

model.gauss_constant * exp(rn*rn*pre1 + rn*gn*pre2 + gn*gn*pre3)

This made it approx. 30 % faster. The pre-variables were moved out
to the color_alg_params struct. However, not enough.

Now we take the logarithm of res. We get the calculation

log(model.gauss_constant) + rn*rn*pre1 + rn*gn*pre2 + gn*gn*pre3

The threshold test is then set to log(model.gauss_thresh)
We then move the log(model.gauss_constant) out to the threshold instead.
This way, that addition is only done once for every image, instead of once
for every pixel in every image (approx 300k additions saved).

Thus the final calculation can be seen below in the code. In total, over 6 times faster
than it was before any optimizations. Everything that could be precalculated was moved
out to the precalc() method of the color_alg_params struct.

NOTE:
reinitializes output
*/
void ColorDetectionNode::multiGaussian(const cv::Mat& src, cv::Mat& output,
                                       const color_alg_params& model) {
    output = cv::Mat(src.rows,src.cols,CV_32F);
    for(int row = 0; row < src.rows; ++row) {
        for(int col = 0; col < src.cols; ++col) {
            const cv::Vec2f& tmp = src.at<cv::Vec2f>(row,col);
            double rn = tmp.val[0]-model.color_model.mu[0];
            double gn = tmp.val[1]-model.color_model.mu[1];
            double res = rn*rn*model.pre1 + rn*gn*model.pre2 + gn*gn*model.pre3;
            //double res = log_const + rn*rn*pre1 + rn*gn*pre2 + gn*gn*pre3;
            /*double res = model.gauss_constant *
                       std::exp(-0.5 * (rn*rn*model.sigma_inv[0][0] +
                                        2*rn*gn*model.sigma_inv[0][1] +
                                        gn*gn*model.sigma_inv[1][1]));
*/

            output.at<float>(row,col) = (float) res;
        }
    }

}

/**
Sorting function for contours. Sorts in descending order.
*/
bool ColorDetectionNode::contourSort(std::vector<cv::Point> c1, std::vector<cv::Point> c2) {
    return cv::contourArea(c1) > cv::contourArea(c2);
}

/**
Update. Run once every tick.
*/
void ColorDetectionNode::update() {
    if((ros::Time::now()-t_rgb).toSec()>1.0) {
        return;
    }

    //convert image to openCV image
    cv_bridge::CvImagePtr cv_ptr = convertImage(camera_img_raw, sensor_msgs::image_encodings::RGB8);

    //blur image
    cv::Mat blurred = cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);
    cv::blur(cv_ptr->image,blurred,cv::Size(blur_size,blur_size));

    //convert to rg-chromasity image.
    cv::Mat rg_chrom_img;
    rgb2rgChromasity(blurred,rg_chrom_img);

    //calculate contours for each model
    std::vector<int> biggest_contours(models.size());
    std::vector<cv::Point2i> contour_centers(models.size());
    for(int i = 0; i < models.size(); ++i) {
        //calculate gaussian values
        cv::Mat gauss_img;
        multiGaussian(rg_chrom_img,gauss_img,models[i]);

        //make a thresholded binary image
        cv::Mat thresh;
        cv::threshold(gauss_img, thresh, models[i].gauss_thresh, 1, cv::THRESH_BINARY);

        //convert to a binary CV_8UC1 image. Needed for contour algorithm.
        cv::Mat binary = cv::Mat::zeros(gauss_img.rows, gauss_img.cols, CV_8UC1);
        thresh.convertTo(binary,CV_8UC1);

        //do contouring
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;

        cv::findContours(binary, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
        std::sort(contours.begin(), contours.end(), &ColorDetectionNode::contourSort);

        //save biggest contour size
        if(contours.size() > 0) {
            biggest_contours[i] = contours[0].size();
            cv::Scalar means = cv::mean(contours[0]);
            contour_centers[i] = cv::Point2i((int) means[0],(int) means[1]);
        } else {
            biggest_contours[i] = 0;
        }


        //if(i == 0) {
            //cv::Mat scaled_img;
            //scaleImage(gauss_img,scaled_img);
            //cv::imshow(colors[i],scaled_img);

        if(i == 5) {
          //  cv::imshow(colors[i],thresh);
            ROS_INFO_STREAM("Contour size of model index " << i << ": " << biggest_contours[i]);
            ROS_INFO_STREAM("Contour center: (row: " << contour_centers[i].y << ", col: " << contour_centers[i].x);
            ROS_INFO_STREAM("Depth there: " << getDepth(contour_centers[i].y, contour_centers[i].x));
            /*//cv::Mat cont_img = cv::Mat::zeros(gauss_img.rows,gauss_img.cols,CV_8UC3);
            //cv::drawContours(cont_img,contours,-1,(0,255,0),3);
            //cv2.drawContours(img, [cnt], 0, (0,255,0), 3)
            cv::Mat cont_img = cv::Mat::zeros(gauss_img.rows,gauss_img.cols,CV_32F);
            //for(int j = 0; j < contours.size(); ++j) {
            int j = 0;
            if(contours.size() > 0)
                for(int k = 0; k < contours[j].size(); ++k) {
                    cont_img.at<float>(contours[j][k].y, contours[j][k].x) = 1;
                }
            //}

            cv::imshow("contours", cont_img);*/
            /*if(contours.size() > 0 && contours[0].size() < 100) {
                for(int k = 0; k < contours[0].size(); ++k) {
                    ROS_INFO_STREAM("(" << contours[0][k].y << "," << contours[0][k].x << ")");
                    //cont_img.at<float>(contours[j][k].y, contours[j][k].y) = 1;
                }
            }*/
        }


        //}


        //cv::imshow("thresh", thresh);



    }
    //cv::waitKey(3);
    //ROS_INFO("ROUND");


    //create msg
    color_detection::colors_detected msg;
    msg.blue = getNewColorStatus();
    msg.green = getNewColorStatus();
    msg.red = getNewColorStatus();
    msg.yellow = getNewColorStatus();
    msg.orange = getNewColorStatus();
    msg.purple = getNewColorStatus();

    //classify
    if(biggest_contours[0] >= models[0].min_contour_size) {
        msg.blue.found = true;
        msg.blue.col = contour_centers[0].x;
        msg.blue.row = contour_centers[0].y;
        msg.blue.depth = getDepth(contour_centers[0].y,contour_centers[0].x);
    }

    if(biggest_contours[1] >= models[1].min_contour_size) {
        msg.green.found = true;
        msg.green.col = contour_centers[1].x;
        msg.green.row = contour_centers[1].y;
        msg.green.depth = getDepth(contour_centers[1].y,contour_centers[1].x);
    }

    if(biggest_contours[2] >= models[2].min_contour_size) {
        msg.red.found = true;
        msg.red.col = contour_centers[2].x;
        msg.red.row = contour_centers[2].y;
        msg.red.depth = getDepth(contour_centers[2].y,contour_centers[2].x);
    }

    if(biggest_contours[3] >= models[3].min_contour_size) {
        msg.yellow.found = true;
        msg.yellow.col = contour_centers[3].x;
        msg.yellow.row = contour_centers[3].y;
        msg.yellow.depth = getDepth(contour_centers[3].y,contour_centers[3].x);
    }

    if(biggest_contours[4] >= models[4].min_contour_size) {
        msg.orange.found = true;
        msg.orange.col = contour_centers[4].x;
        msg.orange.row = contour_centers[4].y;
        msg.orange.depth = getDepth(contour_centers[4].y,contour_centers[4].x);
    }

    if(biggest_contours[5] >= models[5].min_contour_size) {
        msg.purple.found = true;
        msg.purple.col = contour_centers[5].x;
        msg.purple.row = contour_centers[5].y;
        msg.purple.depth = getDepth(contour_centers[5].y,contour_centers[5].x);
    }


    //publish message
    classifier_publisher.publish(msg);


    //classify
    //for(int i = 0; i < biggest_contours.size(); ++i) {


        //ROS_INFO_STREAM("Contour size of model index " << i << ": " << biggest_contours[i]);
    //}
}

/**
Get the distance to the given point. If NaN, return -1.

This code is pretty damn cryptic. At first the function returned dist
immediatly after initializing it. However, I noticed it was NaN quite often,
as the depth camera does not register all points all the time. So because of
that I added some code to check the surroundings of the given point.

NOTE: to check if a float f is NaN:
if(f != f) {
    //f is NaN
}

Basically:

1. get depth at the (row,col) point.
2. if NaN, iterate on points in a square around the middle, from row-thresh to row+thresh
   and col-thresh to col+thresh. The first non-NaN value is returned.
3. Two if-statements at the beginning of each loop check that we don't look out of bounds of the image.
4. If still no non-NaN found, return -1.
*/
float ColorDetectionNode::getDepth(int row, int col) {
    cv_bridge::CvImagePtr cv_ptr = convertImage(camera_depth_raw,sensor_msgs::image_encodings::TYPE_32FC1);
    float dist = cv_ptr->image.at<float>(row,col);
    if(dist != dist) {
        //NaN check, if we get here dist is NaN
        dist = -1;
        int thresh = 3;
        for(int i = -thresh; i <= thresh; ++i) {
            if(i < 0 || i >= cv_ptr->image.rows)
                continue;
            for(int j = -thresh; j <= thresh; ++j) {
                if(j < 0 || j >= cv_ptr->image.cols)
                    continue;
                float tmp = cv_ptr->image.at<float>(row+i,col+j);
                if(!(tmp != tmp)) {
                    return tmp;
                }
            }
        }
    }
    return dist;
}

/**
Returns a freshly initialized  color_status message object.
*/
color_detection::color_status ColorDetectionNode::getNewColorStatus() {
    color_detection::color_status status;
    status.found = false;
    status.col = -1;
    status.row = -1;
    status.depth = 0.0f;
    return status;
}

/**
Read parameters for model and algorithm parameters for color color_model_name.
Every value is put in the given struct &cap. Assumed that it is just initialized with default
constructor.
  */
void ColorDetectionNode::readModel(ros::NodeHandle n, std::string color_model_name,color_alg_params &cap) {
    //read color model
    VisionModels::color_model_vardim<double> color_model(2);
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
    ROSUtil::getParam(n,"/models/" + color_model_name + "/min_contour_size", cap.min_contour_size);
    ROSUtil::getParam(n,"/models/" + color_model_name + "/gauss_thresh", cap.gauss_thresh);
    ROS_INFO_STREAM("Min contour size is " << cap.min_contour_size);
    ROS_INFO_STREAM("Gauss threshold is " << cap.gauss_thresh);

    //precalculate inv_sigma
    cap.precalc_vars();

    /*
    ROSUtil::getParam(n,"/models/" + color_model_name + "/blur_kernel_size", cap.blur_size);
    ROSUtil::getParam(n,"/models/" + color_model_name + "/lines_col", cap.lines_col);
    ROSUtil::getParam(n,"/models/" + color_model_name + "/lines_row", cap.lines_row);
    ROSUtil::getParam(n,"/models/" + color_model_name + "/thresh_row", cap.thresh_row);
    ROSUtil::getParam(n,"/models/" + color_model_name + "/thresh_col", cap.thresh_col);


    ROS_INFO_STREAM("Blur kernel size set to (" << blur_size << "," << blur_size << ")");
    ROS_INFO_STREAM("lines_col: " << lines_col << " lines_row: " << lines_row
                    << " thresh_col: " << thresh_col << " thresh_row: " << thresh_row);
    */

}

void ColorDetectionNode::readParameters(ros::NodeHandle n) {
    ROSUtil::getParam(n,"/color_detection_params/blur_kernel_size", blur_size);
    ROS_INFO_STREAM("SUCCESSFULLY read parameter blur_kernel_size, set to ("
                    << blur_size << "," << blur_size << ")");
}

void ColorDetectionNode::runNode(ros::NodeHandle handle) {
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

ros::NodeHandle ColorDetectionNode::nodeSetup(int argc, char *argv[]) {
    ros::init(argc, argv, "ColorDetector");
    ros::NodeHandle handle;

    //read and store models

    for(int i = 0; i < numColors; ++i) {
        color_alg_params cap;
        readModel(handle,colors[i],cap);
        models.push_back(cap);
    }

    //read parameters
    readParameters(handle);


    //general ros setup
    t_rgb = ros::Time::now();
    rgb_subscriber = handle.subscribe("/camera/rgb/image_rect_color", 1, &ColorDetectionNode::rgbCallback, this);
    depth_subscriber = handle.subscribe("/camera/depth/image_rect", 1, &ColorDetectionNode::depthCallback, this);
    classifier_publisher = handle.advertise<color_detection::colors_detected>("/vision/color_classifier", 1);
    return handle;
}

ColorDetectionNode::ColorDetectionNode(int argc, char *argv[]) {
    ros::NodeHandle handle = nodeSetup(argc, argv);
    runNode(handle);
}

int main(int argc, char* argv[])
{
    ColorDetectionNode cdn(argc, argv);
}

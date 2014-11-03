#include "pictureTaker.hpp"

cv_bridge::CvImagePtr PictureTakerNode::convertImage() {
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

void PictureTakerNode::rgbCallback(const sensor_msgs::Image::ConstPtr &msg) {
    t_camera = ros::Time::now();
    img = msg;
}

void PictureTakerNode::bgr2rgb(cv_bridge::CvImagePtr cv_ptr) {
    cv::Mat& m = cv_ptr->image;
    for(int row = 0; row < m.rows; ++row) {
        for(int col = 0; col < m.cols; ++col) {
            cv::Vec3b& pixel = m.at<cv::Vec3b>(row,col);
            int tmp = pixel.val[0];
            pixel.val[0] = pixel.val[2];
            pixel.val[2] = tmp;
        }
    }
}

void PictureTakerNode::savePicture() {
    cv_bridge::CvImagePtr cv_ptr = convertImage();
    int i = std::rand();
    std::stringstream ss;
    ss << i << ".jpg";
    std::string s = ss.str();
    if(rgb)
        bgr2rgb(cv_ptr);
    cv::imwrite(s,cv_ptr->image);
    std::cout << "Successfully saved image as " << s << std::endl;
}



ros::NodeHandle PictureTakerNode::nodeSetup(int argc, char* argv[]) {
    ros::init(argc, argv, "PurpleClassifier");
    ros::NodeHandle handle;

    std::srand(std::time(NULL));
    rgb_subscriber = handle.subscribe("/camera/rgb/image_rect_color", 1, &PictureTakerNode::rgbCallback, this);

    return handle;
}

void PictureTakerNode::runNode(ros::NodeHandle handle) {
    char input;
    while(true) {
        std::cout << "Any character to take picture, q to quit >";
        std::cin >> input;
        if(input == 'q')
            break;
        ros::spinOnce();
        savePicture();
    }
}

PictureTakerNode::PictureTakerNode(int argc, char* argv[]) {
    rgb = true;
    ros::NodeHandle handle = nodeSetup(argc, argv);
    runNode(handle);

}

int main(int argc, char* argv[])
{
    PictureTakerNode ptn(argc, argv);
}

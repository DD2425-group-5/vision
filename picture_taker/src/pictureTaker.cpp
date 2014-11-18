#include "pictureTaker.hpp"

using std::cout;
using std::endl;

const std::string help = "Picture taker - save images from a ROS image stream.\n\n" \
	"USAGE\n"\
	"\t\t rosrun picture_taker pictureTaker [OPTIONS]\n"\
	"OPTIONS\n"\
	"\t -s \t Save all images from the image stream without waiting for user input. Must provide a rate in hz at which to save.\n\n"\
	"\t -t \t Topic which images are being streamed to.\n\n"\
	"\t -o \t Directory in which to save images.\n\n"\
	"\t -i \t Show the image window while the program is running. Does not work with depth streams.";

cv_bridge::CvImagePtr PictureTakerNode::convertImage() {
    cv_bridge::CvImagePtr cv_ptr;
    
    try {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return cv_ptr;
    }
    
    if (display){
	cv::namedWindow("img", CV_WINDOW_AUTOSIZE);
	cv::imshow("img", cv_ptr->image);
	cv::waitKey(1);
    }

    return cv_ptr;
}

void PictureTakerNode::rgbCallback(const sensor_msgs::Image::ConstPtr &msg) {
    t_camera = ros::Time::now();
    img = msg;
}

/* Swaps red and blue channels */
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

    std::string time = SysUtil::getDateTimeString();
    unsigned long nsec = ros::Time::now().toNSec();
    std::stringstream ss;
    ss << SysUtil::fullDirPath(outDir);
    ss << "img_";
    ss << time;
    ss << "_";
    ss << nsec;
    ss << ".jpg";
    std::string out = ss.str();
    //convert to rgb. Don't need to do this if reading image as RGB in conversion
    if(rgb)
        bgr2rgb(cv_ptr);
    
    cv::imwrite(out, cv_ptr->image);
    std::cout << "Successfully saved image as " << out << std::endl;
}

void PictureTakerNode::runNode() {
    char input;
    ros::Rate loop_rate(streamRate);
    ros::Duration(1).sleep(); // allow camera time to initialise

    while(ros::ok()) {
	// if not saving all stream images, ask for user input
	if (!stream) {
	    std::cout << "Any character to take picture, q to quit >";
	    std::cin >> input;
	    if(input == 'q')
		break;
	} else {
	    loop_rate.sleep();
	}
	
        ros::spinOnce(); //this needs to be here, otherwise ros never calls rgbCallback.
        savePicture();
    }
}

PictureTakerNode::PictureTakerNode(int argc, char* argv[], std::string outd, std::string topc, bool disp, bool strm, int strmhz) {
    outDir = outd;
    topic = topc;
    rgb = true;
    display = disp;
    stream = strm;
    streamRate = strmhz;

    ros::init(argc, argv, "picture_taker");
    ros::NodeHandle handle;
    
    std::srand(std::time(NULL)); //initialize seed for name generation.
    rgb_subscriber = handle.subscribe(topic, 1, &PictureTakerNode::rgbCallback, this);

    runNode();
}

int main(int argc, char* argv[])
{
    std::string outDir = "";
    std::string topic = "/camera/rgb/image_rect_color"; // default topic
    bool display = false;
    bool stream = false;
    int streamRate;
    char c;
    while((c = getopt(argc, argv, "s:t:o:i")) != -1){
	switch (c){
	case 's':
	    stream = true;
	    streamRate = atoi(optarg);
	    break;
	case 't':
	    topic = std::string(optarg);
	    break;
	case 'i':
	    display = true;
	    break;
	case 'o':
	    outDir = std::string(optarg);
	    break;
	case '?':
	    cout << "Don't know option " << optopt << endl;
	    std::exit(1);
	}
    }
    
    PictureTakerNode ptn(argc, argv, outDir, topic, display, stream, streamRate);
}

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv/highgui.h>
#include <string>

using std::string;
using std::cout;

using namespace cv;

int main(int argc, char *argv[])
{
    if (argc < 2) {
	cout << "Please provide a directory from which to read images to classify.";
    }
    return 0;
}

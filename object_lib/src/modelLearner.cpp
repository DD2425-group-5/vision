#include <ros/ros.h>
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv/highgui.h>
#include <sysutil/sysutil.hpp>
#include <mathutil/math.hpp>

using namespace cv;
using std::cout;
using std::string;
using std::vector;
using std::endl;

int main(int argc, char *argv[]) {
    vector<SysUtil::DirContents> content = SysUtil::getDirContents(argv[1]);

    for (size_t dirNum = 0; dirNum < content.size(); dirNum++) {
	SysUtil::DirContents dc = content[dirNum];
	cout << content[dirNum].path << endl;

	// Keep track of the mean and stdeviation averages for all images
	Scalar meanSum, stdevSum;

	cout << meanSum << ", "<< stdevSum << endl;
	for (size_t fileNum = 0; fileNum < dc.files.size(); fileNum++) {
//	    cout << dc.files[fileNum] << endl;
	    
	    Mat img = imread(dc.files[fileNum], CV_LOAD_IMAGE_COLOR);

	    // Use a threshold to get a mask in order to
	    // extract the relevant pixels from the image
	    Mat m, gray, size;
	    resize(img, size, Size(), 0.2, 0.2);
	    cvtColor(size, gray, CV_RGB2GRAY);
	    threshold(gray, m, 0, 255, THRESH_BINARY | THRESH_OTSU);

	    // namedWindow("d", CV_WINDOW_AUTOSIZE);
	    // imshow("d", m);
	    // waitKey(0);

	    // for this image
	    Scalar mean, stdev;
	    meanStdDev(size, mean, stdev, m);
	    cout << "Mean: " << mean << "Stdev: " << stdev << endl;
	    meanSum = meanSum + mean;
	    stdevSum = stdevSum + stdev;
	}
	int size = dc.files.size();
	// Dividing by scalars is screwy, so do it manually instead.
	cout << "Final mean: " << meanSum << " Final stdev: " << stdevSum << endl;
	Scalar fmean = Scalar(meanSum.val[0]/size, meanSum.val[1]/size, meanSum.val[2]/size);
	Scalar fstd = Scalar(stdevSum.val[0]/size, stdevSum.val[1]/size, stdevSum.val[2]/size);
	cout << "Final mean: " << Scalar(meanSum.val[0]/size, meanSum.val[1]/size, meanSum.val[2]/size)
	     << " Final stdev: " << Scalar(stdevSum.val[0]/size, stdevSum.val[1]/size, stdevSum.val[2]/size) << endl;
    }
    
    return 0;
}


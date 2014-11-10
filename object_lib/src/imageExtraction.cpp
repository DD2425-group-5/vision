#include "imageExtraction.hpp"

// Bad practice!
using namespace cv;
using namespace std;

/**
 * Main function to extract objects from images. Gets a list of files to
 * process, and then passes the files to the extraction algorithm. The resulting
 * images are then placed in the output directory with the directory names of
 * the original locations preserved.
 */
void extractObjects(string inputDir, string outputDir) {
    if (!SysUtil::isDir(inputDir)) {
	ROS_INFO("Input directory %s is not a directory.", inputDir.c_str());
	exit(1);
    }

    if (SysUtil::isDir(outputDir)){
	if (SysUtil::queryUserYN(string(outputDir + " already exists. Write to it anyway? (y/n)"))){
	    cout << "Extracted objects will be written to directories in " << outputDir << endl;
	} else {
	    cout << "OK, exiting." << endl;
	    std::exit(0);
	}
    } else {
	cout << string(outputDir + " is not a directory. Creating.") << endl;
	mkdir(outputDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH);
    }
    
    vector<SysUtil::DirContents> dirsToProcess = SysUtil::getDirContents(inputDir);

    // Go over each directory
    for (size_t dirNum = 0; dirNum < dirsToProcess.size(); dirNum++) {
	SysUtil::DirContents currentDir = dirsToProcess[dirNum];
	cout << "Processing directory " << currentDir.path << "..." << endl;

	string outputPath = SysUtil::fullDirPath(outputDir);
	// the directory being processed had no subdirs, so need to add its
	// basename to the output path in order to create the correct output
	// directory.
	if (currentDir.isTop) {
	    outputPath = outputPath + SysUtil::removeBaseName(inputDir);
	}
	
	// make the directory into which the files will be placed, opencv
	// doesn't create directories. 
	if (!SysUtil::isDir(outputPath)) {
	    if (SysUtil::queryUserYN(string(outputPath + " already exists. Files in the directory might be overwritten. Write to the directory? (y/n)"))){
		// if answer is y, create the directory and do the extraction
		cout << "Creating directory " << outputPath << endl;
		mkdir(outputPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH);
	    } else {
		// If the user replies n to the question, skip processing this directory.
		cout << "OK, skipping directory " << currentDir.path << endl;
		continue;
	    }
	}

	// For each file in the current directory
	for (size_t fileNum = 0; fileNum < currentDir.files.size(); fileNum++) {
	    // Extract the blob and then write it to the output directory,
	    // preserving its original path below the input directory.
	    // cout << "Processing " << *fileIterator << endl;
	    // cout << "Will output to " << insertSuffix(outputPath + removeBaseName(*fileIterator), "_object") << endl;
	    string fileName = currentDir.files[fileNum];
	    // Replace the input directory with the output directory in the filename
	    Mat extract = threshContour(fileName);
	    imwrite(SysUtil::insertSuffix(outputPath + SysUtil::removeBaseName(fileName), "_object"), extract);
	    // \r at the beginning of the line returns to the beginning, using
	    // flush at the end insted of endl allows overwriting the stuff
	    // written on the current line. Do this to prevent lots of lines of
	    // output.
	    cout << "\rProcessed " << fileNum + 1
		 << " of " << currentDir.files.size()
		 << " images." << flush;
	}
	cout << endl;
    }
}

bool contourSort(vector<Point> c1, vector<Point> c2) {
    return contourArea(c1) > contourArea(c2);
}

// int threshSet;
// int maxThresh = 255;
// Mat grayImg;

// void threshCallback(int, void*) {
//     Mat thresh;
//     threshold(grayImg, thresh, threshSet, 255, THRESH_BINARY_INV);
//     imshow("source", thresh);
// }

Mat threshContour(string fileName) {
    Mat im = imread(fileName, CV_LOAD_IMAGE_COLOR);

    if (!im.data) {
	ROS_INFO("Could not load image %s", fileName.c_str());
    }
    
    Mat img = im;
//    resize(im, img, Size(), 1, 1);
    Mat gray;
    cvtColor(img, gray, CV_RGB2GRAY);

    // namedWindow("source", CV_WINDOW_AUTOSIZE);
    // imshow("source", grayImg);
    // createTrackbar("Threshold", "source", &threshSet, maxThresh, threshCallback);
    // threshCallback(0, 0);
    // waitKey(0);

    Mat thresh;
    threshold(gray, thresh, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);
//    threshold(gray, thresh, threshSet, 255, THRESH_BINARY_INV); 
    
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    
    findContours(thresh, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    sort(contours.begin(), contours.end(), contourSort);

    Mat drawing = Mat::zeros(img.size(), CV_8UC1);
    Scalar colour = Scalar(1,1,1);
    drawContours(drawing, contours, 0, colour, CV_FILLED);

    //  RNG rng;    
    // Mat cont;
    // img.copyTo(cont);
    // for (int i = 0; i < contours.size(); i++) {
    // 	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
    // 	drawContours(cont, contours, i, color, 2, 8, hierarchy, 0, Point());
    // }
    // namedWindow("cont", CV_WINDOW_AUTOSIZE);
    // imshow("cont", cont);
    // waitKey(0);
    
    Mat extracted;
    img.copyTo(extracted, drawing);

    return extracted;
}

int main(int argc, char *argv[]) {
    if (argc < 3) {
	cout << "Please provide an directory to process, and a directory "\
	    "into which to output processed images." << endl;
    }
    extractObjects(argv[1], argv[2]);
}

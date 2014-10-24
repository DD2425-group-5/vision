#include "imageExtraction.hpp"

using namespace cv;
using namespace std;

/**
 * Main function to extract objects from images. Gets a list of files to
 * process, and then passes the files to the extraction algorithm. The resulting
 * images are then placed in the output directory with the directory names of
 * the original locations preserved.
 */
void extractObjects(string inputDir, string outputDir) {
    if (!isDir(inputDir)) {
	ROS_INFO("Input directory %s is not a directory.", inputDir.c_str());
	exit(1);
    }

    if (isDir(outputDir)) {
	ROS_INFO("%s already exists. Write to this directory? (y/n)", outputDir.c_str());
	string reply;
	bool done = false;
	while (!done) {
	    cin >> reply;
	    if (reply.compare("y") == 0 || reply.compare("Y") == 0) {
		done = true;
	    } else if (reply.compare("n") == 0 || reply.compare("N") == 0) {
		ROS_INFO("OK, exiting.");
		exit(0);
	    } else {
		ROS_INFO("Please reply with y or n.");
	    }
	}
    } else {
	ROS_INFO("%s is not a directory. Creating.", outputDir.c_str());
	mkdir(outputDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH);
    }

    vector<DirContents> dirsToProcess = getFilesToProcess(inputDir);

    // Go over each directory
    for (int dirNum = 0; dirNum < dirsToProcess.size(); dirNum++) {
	DirContents currentDir = dirsToProcess[dirNum];
	string dir = currentDir.path;
	string outputPath = dir;

	// make the directory into which the files will be placed, opencv
	// doesn't create directories. Remove the final slash from the output
	if (outputPath.compare("") != 0) {
	    outputPath.replace(0, inputDir.size(), outputDir);
	} else {
	    outputPath.replace(0, inputDir.size(), outputDir + removeBaseName(inputDir));
	    dir = inputDir;
	}
	std::cout << outputPath << std::endl;
	if (!isDir(outputPath)) {
	    cout << "Creating directory " << outputPath << endl;
	    mkdir(outputPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH);
	}
	cout << "Processing directory " << dir << "..." << endl;
	// For each file in the current directory
	for (int fileNum = 0; fileNum < currentDir.files.size(); fileNum++) {
	    // Extract the blob and then write it to the output directory,
	    // preserving its original path below the input directory.
	    // cout << "Processing " << *fileIterator << endl;
	    // cout << "Will output to " << insertSuffix(outputPath + removeBaseName(*fileIterator), "_object") << endl;
	    string fileName = currentDir.files[fileNum];
	    // Replace the input directory with the output directory in the filename
	    Mat extract = threshContour(fileName);
	    imwrite(insertSuffix(outputPath + removeBaseName(fileName), "_object"), extract);
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
    Mat im;
    
    im = imread(fileName, CV_LOAD_IMAGE_COLOR);

    if (!im.data) {
	ROS_INFO("Could not load image %s", fileName.c_str());
    }
    
    Mat img;
    resize(im, img, Size(), 0.2, 0.2);
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

vector<DirContents> getFilesToProcess(string dirName) {
    vector<DirContents> dirs;

    DirContents top = listDir(dirName);
    
    if (top.dirs.size() == 0) {
	// If there are no directories, assume you want to process files in the
	// directory
	top.path = "";
	dirs.push_back(top);
    } else {
	// Otherwise, assume that the maximum depth of the directory structure
	// is 1, and extract the file names out of the directories below the one
	// passed as a parameter.
	for (int dir = 0; dir < top.dirs.size(); dir++){
	    DirContents sub = listDir(top.dirs[dir]);
	    if (sub.files.size() != 0) {
		dirs.push_back(sub);
	    }
	}
    }
    return dirs;
}

int main(int argc, char *argv[]) {
    extractObjects(argv[1], argv[2]);
}

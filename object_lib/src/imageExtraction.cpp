#include "imageExtraction.hpp"

using namespace cv;


bool isType(std::string path, mode_t mode) {
    struct stat s;
    if (stat(path.c_str(), &s) == 0) {
	if (s.st_mode & mode) {
	    return true;
	} else {
	    return false;
	}
    } else {
	return false;
    }
}

bool isDir(std::string path) {
    return isType(path, S_IFDIR);
}

bool isFile(std::string path) {
    return isType(path, S_IFREG);
}

/**
 * Insert a suffix at the end of a filename which has an extension. The suffix
 * will be placed before the extension.
 */
std::string insertSuffix(std::string fileName, std::string suffix) {
    int dotInd = fileName.find_last_of(".");
    std::string raw = fileName.substr(0, dotInd);
    std::string ext = fileName.substr(dotInd);
    
    return raw + suffix + ext;
}

/**
 * Main function to extract objects from images. Gets a list of files to
 * process, and then passes the files to the extraction algorithm. The resulting
 * images are then placed in the output directory with the directory names of
 * the original locations preserved.
 */
void extractObjects(std::string inputDir, std::string outputDir) {
    if (!isDir(inputDir)) {
	ROS_INFO("Input directory %s is not a directory.", inputDir.c_str());
	std::exit(1);
    }

    if (isDir(outputDir)) {
	ROS_INFO("%s already exists. Write to this directory? (y/n)", outputDir.c_str());
	std::string reply;
	bool done = false;
	while (!done) {
	    std::cin >> reply;
	    if (reply.compare("y") == 0 || reply.compare("Y") == 0) {
		done = true;
	    } else if (reply.compare("n") == 0 || reply.compare("N") == 0) {
		ROS_INFO("OK, exiting.");
		std::exit(0);
	    } else {
		ROS_INFO("Please reply with y or n.");
	    }
	}
    } else {
	ROS_INFO("%s is not a directory. Creating.", outputDir.c_str());
	mkdir(outputDir.c_str(), S_IRWXU | S_IRWXG | S_IROTH);
    }

    std::map<std::string, std::vector<std::string> > dirMap = getFilesToProcess(inputDir);
    std::map<std::string, std::vector<std::string> >::iterator mapIterator;
    std::vector<std::string>::iterator fileIterator;
    // Go over each directory
    for (mapIterator = dirMap.begin(); mapIterator != dirMap.end(); ++mapIterator) {

	std::string dir = mapIterator->first;
	std::vector<std::string> files = mapIterator->second;
	// make the directory into which the files will be placed, opencv
	// doesn't create directories.
	std::string dirPath = std::string(outputDir + "/" + dir);
	if (!isDir(dirPath)) {
	    std::cout << "Creating directory " << dirPath << std::endl;
	    mkdir(dirPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH);
	}
	std::cout << "Processing directory " << dir << "..." << std::endl;
	// For each file in the current directory
	for (fileIterator = files.begin(); fileIterator != files.end(); ++fileIterator){
	    // Extract the blob and then write it to the output directory,
	    // preserving its original path. Need to reconstruct the sub-path
	    // of the input image.
	    Mat extract = detectBlob(inputDir + dir + "/" + *fileIterator);
	    imwrite(outputDir + "/" + dir + "/" + insertSuffix(*fileIterator, "_object"), 
		    extract);
	    // \r at the beginning of the line returns to the beginning, using
	    // flush at the end insted of endl allows overwriting the stuff
	    // written on the current line. Do this to prevent lots of lines of
	    // output.
	    std::cout << "\rProcessed " << fileIterator - files.begin() + 1
		      << " of " << files.end() - files.begin() 
		      << " images." << std::flush;
	}
	std::cout << std::endl;
    }
}

bool contourSort(std::vector<Point> c1, std::vector<Point> c2) {
    return contourArea(c1) > contourArea(c2);
}

Mat detectBlob(std::string fileName) {
    Mat im;
    
    im = imread(fileName, CV_LOAD_IMAGE_COLOR);

    if (!im.data) {
	ROS_INFO("Could not load image %s", fileName.c_str());
    }
    
    Mat img;
    resize(im, img, Size(), 0.2, 0.2);
    Mat gray;
    cvtColor(img, gray, CV_RGB2GRAY);
    Mat thresh;
    threshold(gray, thresh, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);
    
    std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    
    findContours(thresh, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    std::sort(contours.begin(), contours.end(), contourSort);

    Mat drawing = Mat::zeros(img.size(), CV_8UC1);
    Scalar colour = Scalar(1,1,1);
    drawContours(drawing, contours, 0, colour, CV_FILLED);
    
    Mat extracted;
    img.copyTo(extracted, drawing);

    return extracted;
}

std::vector<std::string> getDirFiles(std::string dirName) {
    DIR* dir;
    struct dirent* ent;
    std::vector<std::string> fileList;
    // Go through the directory and put all the directories in it into a list. These
    // are assumed to be the directories containing images to be processed
    if ((dir = opendir(dirName.c_str())) != NULL) {
	while ((ent = readdir(dir)) != NULL) {
	    // not interested in the current or parent dirs
	    if (strcmp(ent->d_name, ".") == 0 || strcmp(ent->d_name, "..") == 0){
		continue;
	    }
	    fileList.push_back(std::string(ent->d_name));
	}
	closedir(dir);
    } else {
	// Couldn't access directory
	ROS_INFO("Error processing %s:", dirName.c_str());
	perror("");
    }
    
    return fileList;
}

/**
 * Mixing c and c++ is bad, but unavoidable here.
 */ 
std::map<std::string, std::vector<std::string> > getFilesToProcess(std::string dirName) {


    std::map<std::string, std::vector<std::string> > dirFileMap;
    DIR* dir;
    struct dirent* ent;
    // Go through the directory and put all the directories in it into a list. These
    // are assumed to be the directories containing images to be processed
    if ((dir = opendir(dirName.c_str())) != NULL) {
	while ((ent = readdir(dir)) != NULL) {
	    // not interested in the current or parent dirs, or files in this dir
	    if (isFile(dirName + std::string(ent->d_name)) 
		|| strcmp(ent->d_name, ".") == 0 
		|| strcmp(ent->d_name, "..") == 0){
		continue;
	    }
	    std::string subdName = std::string(ent->d_name);
	    // Insert the directory and its files into the map to be processed. Don't need the
	    // full path in the map, reconstruct it later.
	    dirFileMap.insert(std::pair<std::string, std::vector<std::string> >(subdName, getDirFiles(dirName + subdName)));
	}
	closedir(dir);
    } else {
	// Couldn't access directory
	perror("");
	ROS_INFO("Error processing %s", dirName.c_str());
    }

    return dirFileMap;
}

int main(int argc, char *argv[]) {
    extractObjects(argv[1], argv[2]);
}

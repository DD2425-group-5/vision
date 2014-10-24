#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv/highgui.h>
#include <iostream>
#include <algorithm>
#include <dirent.h>
#include <sys/stat.h>
#include <string>
#include <iostream>
#include <map>

struct DirContents {
    std::string path;
    std::vector<std::string> dirs;
    std::vector<std::string> files;
};

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
 * Removes the trailing slash from a directory path if there is one.
 */
std::string cleanDirPath(std::string path) {
    std::string dir = path;
    if (path[path.size() - 1] == '/'){
	dir = path.substr(0, dir.size() - 1);
    }
    return dir;
}

/**
 * Removes the base name from a path, leaving either the directory or the 
 * file name.
 */
std::string removeBaseName(std::string path) {
    if (isFile(path)){
	int lastDir = path.find_last_of("/");
	return path.substr(lastDir);
    } else if (isDir(path)){
	std::string cleanPath = cleanDirPath(path);
	int lastDir = cleanPath.find_last_of("/");
	return cleanPath.substr(lastDir + 1);
    }
    return path;
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


DirContents listDir(std::string path) {
    DirContents c;
    c.path = cleanDirPath(path);
    DIR* dir;
    struct dirent* ent;
    if ((dir = opendir(path.c_str())) != NULL) {
	while ((ent = readdir(dir)) != NULL) {
	    // ignore current and parent dirs
	    if (strcmp(ent->d_name, ".") == 0 
		|| strcmp(ent->d_name, "..") == 0){
		continue;
	    }
	    std::string fullPath = c.path + "/" + std::string(ent->d_name);
	    if (isFile(fullPath)) {
		c.files.push_back(fullPath);
	    } else if (isDir(fullPath)) {
		c.dirs.push_back(fullPath);
	    }
	}
    } else {
	// Couldn't access directory
	perror("");
	ROS_INFO("Error processing %s", path.c_str());
    }

    return c;
}


void extractObjects(std::string inputDir, std::string outputDir);
bool contourSort(std::vector<cv::Point> c1, std::vector<cv::Point> c2);
cv::Mat threshContour(std::string fileName);
std::vector<DirContents> getFilesToProcess(std::string dirName);
std::string insertSuffix(std::string fileName, std::string suffix);

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>
#include <iostream>
#include <algorithm>
#include <dirent.h>
#include <sys/stat.h>
#include <string>
#include <map>

bool isType(std::string path, mode_t mode);
bool isDir(std::string path);
bool isFile(std::string path);
void extractObjects(std::string inputDir, std::string outputDir);
bool contourSort(std::vector<cv::Point> c1, std::vector<cv::Point> c2);
cv::Mat detectBlob(std::string fileName);
std::vector<std::string> getDirFiles(std::string dirName);
std::map<std::string, std::vector<std::string> > getFilesToProcess(std::string dirName);
std::string insertSuffix(std::string fileName, std::string suffix);

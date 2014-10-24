#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv/highgui.h>
#include <iostream>
#include <sysutil.hpp>

void extractObjects(std::string inputDir, std::string outputDir);
bool contourSort(std::vector<cv::Point> c1, std::vector<cv::Point> c2);
cv::Mat threshContour(std::string fileName);
std::vector<SysUtil::DirContents> getFilesToProcess(std::string dirName);

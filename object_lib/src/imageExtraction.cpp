#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>
#include <iostream>





int main(int argc, char *argv[]) {
     for (int i = 0; i < argc; i++) {
	  ROS_INFO("Arg %d: %s", i, argv[i]);
     }

}

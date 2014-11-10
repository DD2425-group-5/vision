cmake_minimum_required(VERSION 2.8.3)
project(color_detection)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport cv_bridge rosutil visionutil)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES color_detection
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

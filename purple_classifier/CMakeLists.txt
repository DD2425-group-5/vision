cmake_minimum_required(VERSION 2.8.3)
project(purple_classifier)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport cv_bridge rosutil visionutil)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES blobdetection
#  CATKIN_DEPENDS roscpp image_transport cv_bridge
#  DEPENDS visionutil
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(purpleClassifier src/purpleClassifierNode.cpp)
target_link_libraries(purpleClassifier ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(purpleClassifier ${catkin_EXPORTED_TARGETS})


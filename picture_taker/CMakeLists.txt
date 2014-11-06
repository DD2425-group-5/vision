cmake_minimum_required(VERSION 2.8.3)
project(picture_taker)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport
cv_bridge sysutil)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES blobdetection
#  CATKIN_DEPENDS roscpp image_transport cv_bridge
#  DEPENDS visionutil
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pictureTaker src/pictureTaker.cpp)
target_link_libraries(pictureTaker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} sysutil)
add_dependencies(pictureTaker ${catkin_EXPORTED_TARGETS})

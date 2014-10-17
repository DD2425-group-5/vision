cmake_minimum_required(VERSION 2.8.3)
project(image_conversion)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport cv_bridge)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_convertion
# CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

add_executable(imageConvertert src/imageConverter.cpp)
target_link_libraries(imageConvertert ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(imageConvertert ${catkin_EXPORTED_TARGETS})


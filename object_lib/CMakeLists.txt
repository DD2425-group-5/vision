cmake_minimum_required(VERSION 2.8.3)
project(object_lib)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES blobdetection
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

add_executable(image_extractor src/imageExtraction.cpp)
target_link_libraries(image_extractor ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(image_extractor ${catkin_EXPORTED_TARGETS})

cmake_minimum_required(VERSION 2.8.3)
project(vision_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs sensor_msgs pcl_msgs)

add_message_files(
  DIRECTORY msg
  FILES
    color_status.msg
    colors_detected.msg
    color_and_cube.msg
    colors_with_shape_info.msg
    object_found.msg
    plane_extracted.msg
)

generate_messages(
  DEPENDENCIES std_msgs sensor_msgs pcl_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES blobdetection
  CATKIN_DEPENDS std_msgs message_runtime sensor_msgs pcl_msgs
#  DEPENDS visionutil
)

include_directories(include ${catkin_INCLUDE_DIRS})

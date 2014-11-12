cmake_minimum_required(VERSION 2.8.3)
project(color_detection)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs message_generation image_transport cv_bridge 
                                        rosutil visionutil)
find_package(OpenCV REQUIRED)

add_message_files( FILES
  colors_detected.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES color_detection
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(colorDetector src/colordetectionnode.cpp)
target_link_libraries(colorDetector ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(colorDetector ${catkin_EXPORTED_TARGETS})

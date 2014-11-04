cmake_minimum_required(VERSION 2.8.3)
project(pcl_methods)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport pcl_ros pcl_conversions)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS roscpp image_transport sensor_msgs pcl_ros pcl_conversions
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(pcl_test src/pcl_test.cpp)
target_link_libraries(pcl_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pcl_test ${catkin_EXPORTED_TARGETS})

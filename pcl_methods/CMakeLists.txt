cmake_minimum_required(VERSION 2.8.3)
project(pcl_methods)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport pcl_ros pcl_conversions pclutil rosutil)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS roscpp image_transport sensor_msgs pcl_ros pcl_conversions
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(plane_extraction src/plane_extraction.cpp)
target_link_libraries(plane_extraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} pclutil)
add_dependencies(plane_extraction ${catkin_EXPORTED_TARGETS})

add_executable(object_clustering src/object_clustering.cpp)
target_link_libraries(object_clustering ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(object_clustering ${catkin_EXPORTED_TARGETS})

add_executable(cubeIdentifier src/cubeidentifiernode.cpp)
target_link_libraries(cubeIdentifier ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(cubeIdentifier ${catkin_EXPORTED_TARGETS})

cmake_minimum_required(VERSION 2.8.3)
project(vision_master)


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs color_detection)

catkin_package(
)


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(visionMaster src/visionmasternode.cpp)
target_link_libraries(visionMaster ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(visionMaster ${catkin_EXPORTED_TARGETS})



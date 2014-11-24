cmake_minimum_required(VERSION 2.8.3)
project(vision_master)


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs message_generation color_detection
    visionutil rosutil)# ras_msgs)
#find_package( REQUIRED COMPONENTS)

add_message_files(
  DIRECTORY msg
  FILES
    object_found.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(visionMaster src/visionmasternode.cpp)
target_link_libraries(visionMaster ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(visionMaster ${catkin_EXPORTED_TARGETS})

add_executable(evidence src/evidencenode.cpp)
target_link_libraries(evidence ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(evidence ${catkin_EXPORTED_TARGETS})

cmake_minimum_required(VERSION 2.8.3)
project(object_lib)

find_package(catkin REQUIRED COMPONENTS roscpp sysutil mathutil)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES blobdetection
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(image_extractor src/imageExtraction.cpp)
target_link_libraries(image_extractor ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} sysutil)
add_dependencies(image_extractor ${catkin_EXPORTED_TARGETS})

add_executable(model_learner src/modelLearner.cpp)
target_link_libraries(model_learner ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} sysutil yaml-cpp)
add_dependencies(model_learner ${catkin_EXPORTED_TARGETS})

add_executable(object_classifier src/objectClassifier.cpp)
target_link_libraries(object_classifier ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(object_classifier ${catkin_EXPORTED_TARGETS})

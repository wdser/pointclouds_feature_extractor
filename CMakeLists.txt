cmake_minimum_required(VERSION 3.0.2)
project(feature_extractor)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(PCL  REQUIRED COMPONENTS common io visualization segmentation)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
include_directories(${JSONCPP_LIBRARIES})

add_executable(feature_extractor src/feature_extractor.cpp src/main.cpp)
target_link_libraries(feature_extractor  ${VTK_LIBRARIES} 
${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${OpenCV_LIBS}
${PCL_VISUALIZATION_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES})

target_link_libraries (feature_extractor  ${JSONCPP_LIBRARIES})

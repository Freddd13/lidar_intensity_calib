cmake_minimum_required(VERSION 3.0.2)
project(intensity_calibration)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package (Glog REQUIRED)
find_package (yaml-cpp REQUIRED)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# link_directories(${YAML_CPP_LIBRARY_DIRS})

add_executable(calib 
  src/main.cpp
  # src/calibrator.cpp
  )
target_link_libraries(calib ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})
# target_link_libraries(k_lidar_preprocess seg ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${GLOG_LIBRARIES})

add_executable(down 
  tools/downsample.cpp
  # src/calibrator.cpp
  )
target_link_libraries(down ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES} ${GFLAGS_LIBRARIES} ${GLOG_LIBRARIES})
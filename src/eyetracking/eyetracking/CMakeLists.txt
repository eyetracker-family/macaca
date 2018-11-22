cmake_minimum_required(VERSION 2.8.3)
project(eyetracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eyetracking_msgs
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  dynamic_reconfigure
  )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(Pure_include_DIRS "/home/macaca/macaca/src/eyetracking/eyetracking/include/pure")
## System dependencies are found with CMake's conventions
find_package(OpenCV 3.3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(TBB REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/PupilParams.cfg
  )

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs eyetracking_msgs dynamic_reconfigure
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${Pure_include_DIRS}
  )
include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake) 

set(tracking_SRC
src/eyetracking_node.cpp
src/ElSe.cpp
src/ExCuSe.cpp
src/PupilDetectionMethod.cpp
src/PuRe.cpp
src/PupilTrackingMethod.cpp
src/PuReST.cpp)

add_executable(${PROJECT_NAME}_node ${tracking_SRC})


link_directories(${OpenCV_LIB_DIR})
target_link_libraries(${PROJECT_NAME}_node
  pupiltracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  )
add_dependencies(${PROJECT_NAME}_node eyetracking_msgs_gencpp)

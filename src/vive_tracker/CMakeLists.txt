cmake_minimum_required(VERSION 2.8.3)
project(vive_tracker)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)
find_package(OpenCV 3.3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vive_tracker
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

set(OpenVR_INCLUDE_DIRS "/home/macaca/macaca/src/vive_tracker/include/vive_tracker")
#set(OpenCV_DIR /opt/ros/kinetic/lib/x86_64-linux-gnu)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenVR_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(OpenVR_LIBS_DIRS "/home/macaca/macaca/linux64")
find_library(LIBOPENVR openvr_api ${OpenVR_LIBS_DIRS})

set(tracker_SRC 
	src/cpTime.cpp
	src/cylinder.cpp
	src/LighthouseTracking.cpp
	src/tracker.cpp )

add_executable(tracker_node ${tracker_SRC})
target_link_libraries(tracker_node ${catkin_LIBRARIES} ${LIBOPENVR} ${OpenCV_LIBS})



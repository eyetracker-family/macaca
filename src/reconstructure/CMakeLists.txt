cmake_minimum_required(VERSION 2.8.3)
project(reconstructure)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs genmsg##
  image_transport
  cv_bridge
serial
  dynamic_reconfigure
  )

#find_package(PCL 1.2 REQUIRED)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
## System dependencies are found with CMake's conventions
find_package(OpenCV 3.3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs dynamic_reconfigure rt pthread serial##
  )


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  )

add_executable(reconstructure_node src/reconstructure.cpp)
link_directories(${OpenCV_LIB_DIR})
target_link_libraries(reconstructure_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
 ${PCL_LIBRARIES}
  )

add_executable(ros_serial_node src/ros_serial.cpp)
target_link_libraries(ros_serial_node
  ${catkin_LIBRARIES}
  )

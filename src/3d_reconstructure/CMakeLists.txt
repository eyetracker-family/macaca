cmake_minimum_required(VERSION 2.8.3)
project(3d_reconstructure)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
	image_transport
	cv_bridge
)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
## System dependencies are found with CMake's conventions
find_package(OpenCV 3.3 REQUIRED)

catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs	image_transport cv_bridge
  )

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake) 

add_executable(${PROJECT_NAME}_node src/reconstructure.cpp)
link_directories(${OpenCV_LIB_DIR})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
 ${PCL_LIBRARIES}
  )

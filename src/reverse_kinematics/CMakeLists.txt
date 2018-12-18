cmake_minimum_required(VERSION 2.8.3)
project(reverse_kinematics)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  serial
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES communication
  CATKIN_DEPENDS roscpp rospy serial std_msgs
#  DEPENDS system_lib
)

set(Inverse_INCLUDE_DIRS "/home/macaca/macaca/src/reverse_kinematics/include/reverse_kinematics")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Inverse_INCLUDE_DIRS}
)

set(inverse_SRC 
	src/foward_kinemat.cpp
	src/inverse_kinemat.cpp
	src/matrix_multiply.cpp
	src/reverse_kinematics.cpp)

add_executable(reverse_node ${inverse_SRC})
target_link_libraries(reverse_node ${catkin_LIBRARIES})


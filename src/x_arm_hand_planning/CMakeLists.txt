cmake_minimum_required(VERSION 2.8.3)
project(x_arm_hand_planning)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  interactive_markers
  moveit_core
  moveit_ros_perception
  moveit_ros_planning_interface
  pluginlib
  roscpp
  rospy
  std_msgs
)

find_package(OpenCV 3.3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES x_arm_hand_grasp_planning
  CATKIN_DEPENDS cmake_modules interactive_markers moveit_core moveit_ros_perception moveit_ros_planning_interface pluginlib roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  "./include/x_arm_hand_grasp_planning/"
)

add_executable(${PROJECT_NAME}_test_random ./src/test_random.cpp ./src/addObject.cpp ./src/collisionCheck.cpp)
add_executable(${PROJECT_NAME}_pose_scan ./src/poseScan.cpp ./src/addObject.cpp ./src/collisionCheck.cpp)

link_directories(${OpenCV_LIB_DIR})
target_link_libraries(${PROJECT_NAME}_test_random ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME}_pose_scan ${catkin_LIBRARIES} ${OpenCV_LIBS})



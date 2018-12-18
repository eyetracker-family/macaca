#include <ros/ros.h>

#include "../include/x_arm_hand_planning/addObject.h"
#include "../include/x_arm_hand_planning/collisionCheck.h"
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>


#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "x_arm_test_random");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveItErrorCode success;

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::MoveGroupInterface group("x_arm");
  group.setPlanningTime(1);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  geometry_msgs::Pose ballPose;
  ballPose.orientation.w=1;
  ballPose.orientation.x=0;
  ballPose.orientation.y=0;
  ballPose.orientation.z=0;
  ballPose.position.x =0;
  ballPose.position.y =0;
  ballPose.position.z =0.66;

  addBall(planning_scene_interface,"base_link","ball1",ballPose,0.03,0);

//  group.setRandomTarget();
  // 开始运动规划，并且让机械臂移动到目标位置
//  group.move();

  // 设置机器人终端的目标位置
  geometry_msgs::Pose target_pose;
  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  double pi = 3.1415926;
  double plan_index = 0;
  std::string fileName="poseSolve.YAML";
  cv::FileStorage fs(fileName, cv::FileStorage::WRITE);

  //generate poteitial oritation
  tf2::Quaternion myOritation;
  tf2::Vector3 seedPosition(0,0,66);
  tf2::Vector3 seedOritationEuler(-pi/2,pi/2,0);


  std::cout<<"plan_index "<<plan_index<<std::endl;
  std::cout<<"position, "<<"x,y,z "<<seedPosition.x()<<" "<<seedPosition.y()<<" "<<seedPosition.z()<<std::endl;
  plan_index++;
  myOritation.setEuler(-pi/2,pi/2,0);
  target_pose.orientation.w = myOritation.w();
  target_pose.orientation.x=myOritation.x();
  target_pose.orientation.y = myOritation.y();
  target_pose.orientation.z = myOritation.z();

  target_pose.position.x =double(seedPosition.x())/100.0;
  target_pose.position.y =double(seedPosition.y())/100.0;
  target_pose.position.z = double(seedPosition.z())/100.0;
  group.setPoseTarget(target_pose);
  success = group.plan(my_plan);
  if(success)
  {
      std::cout<<"plan success"<<std::endl;
  }
  else
  {
      std::cout<<"plan faild"<<std::endl;
  }



  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");   

  //让机械臂按照规划的轨迹开始运动。
  if(!success)
      group.execute(my_plan);

  collisionCheck();

  ros::waitForShutdown();
  return 0;
}

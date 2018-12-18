#include <ros/ros.h>

#include "../include/x_arm_hand_planning/addObject.h"
#include "../include/x_arm_hand_planning/collisionCheck.h"
#include <tf2/LinearMath/Quaternion.h>
#include <iostream>

#include <string>
#include <opencv2/core/core.hpp>

#define X_center 70
#define Y_center 70
#define Z_center 0
#define X_range 140
#define Y_range 140
#define Z_range 70
#define angleSearchN 1
#define angleSearchStepDegree 5
#define pi 3.1415926
#define armLength 66 //cm

bool positionPoseScan(cv::Mat& positionMatrix,
                      cv::Point3i& seedPoint,
                      cv::Point3d& seedOritationEuler,
                      moveit::planning_interface::MoveGroupInterface& group);


bool positionPoseScan(cv::Mat& positionMatrix,
                      cv::Point3i& seedPoint,
                      cv::Point3d& seedOritationEuler,
                      moveit::planning_interface::MoveGroupInterface& group)
{
    if(sqrt(seedPoint.x*seedPoint.x+seedPoint.y*seedPoint.y+seedPoint.z*seedPoint.z)>armLength)
        return false;

    moveit::planning_interface::MoveItErrorCode success;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    geometry_msgs::Pose target_pose;
    tf2::Quaternion currentOritation;

    double yaw,pitch,roll;
    double yawOptimal,pitchOptimal,rollOptimal;
    bool reachable = false;

    yaw = seedOritationEuler.x;
    pitch = seedOritationEuler.y;
    roll = seedOritationEuler.z;

//    std::cout<<"yaw pitch roll "<<yaw<<" "<<pitch<<" "<<roll<<" "<<std::endl;

    double optiLoss = 100;  //minimum this loss
    for(int i =-angleSearchN;i<=angleSearchN;i++)
        for(int j =-angleSearchN;j<=angleSearchN;j++)
            for(int k =-angleSearchN;k<=angleSearchN;k++)
            {
                currentOritation.setEuler(yaw+i*angleSearchStepDegree*pi/180,
                                          pitch+j*angleSearchStepDegree*pi/180,
                                          roll+k*angleSearchStepDegree*pi/180);
                target_pose.orientation.w = currentOritation.w();
                target_pose.orientation.x=currentOritation.x();
                target_pose.orientation.y = currentOritation.y();
                target_pose.orientation.z = currentOritation.z();
                target_pose.position.x =double(seedPoint.x)/100.0;
                target_pose.position.y = double(seedPoint.y)/100.0;
                target_pose.position.z = double(seedPoint.z)/100.0;

                group.setPoseTarget(target_pose);
                success = group.plan(my_plan);

                if(success)
                {
                   if(fabs(my_plan.trajectory_.joint_trajectory.points.back().positions.at(6))<optiLoss)
                   {
                       optiLoss = fabs(my_plan.trajectory_.joint_trajectory.points.back().positions.at(6));
                       yawOptimal = yaw+i*angleSearchStepDegree*pi/180;
                       pitchOptimal = pitch+j*angleSearchStepDegree*pi/180;
                       rollOptimal = roll+k*angleSearchStepDegree*pi/180;

                       int idx[]={seedPoint.x+X_center,seedPoint.y+Y_center,seedPoint.z+Z_center,0};
                       positionMatrix.at<double>(idx)=yawOptimal;
                       idx[3]=1;
                       positionMatrix.at<double>(idx)=pitchOptimal;
                       idx[3]=2;
                       positionMatrix.at<double>(idx)=rollOptimal;

                       reachable =true;
                   }
                }
            }

    if(!reachable)
    {
        std::cout<<"plan fail, point(x,y,z): "<< seedPoint.x <<" "<<seedPoint.y <<" "<<seedPoint.z <<" "<<std::endl;
        return false;
    }
    std::cout<<"plan success, point(x,y,z): "<< seedPoint.x <<" "<<seedPoint.y <<" "<<seedPoint.z <<" "
            <<"oritation(yaw,pitch,roll): "<< yawOptimal<<" "<<pitchOptimal<<" "<<rollOptimal <<std::endl;

    //seed fill
    cv::Point3d subSeedOritationEuler;
    subSeedOritationEuler.x = yawOptimal;
    subSeedOritationEuler.y = pitchOptimal;
    subSeedOritationEuler.z = rollOptimal;

    for(int i =-1;i<=1;i++)
        for(int j =-1;j<=1;j++)
            for(int k =-1;k<=1;k++)
            {
                cv::Point3i subSeedPoint(seedPoint.x+i,seedPoint.y+j,seedPoint.z+k);
                int idSub[]={subSeedPoint.x+X_center,subSeedPoint.y+Y_center,subSeedPoint.z+Z_center,2};
                if(positionMatrix.at<double>(idSub)<-99 &&
                        (subSeedPoint.x+X_center) >= 0 && (subSeedPoint.x+X_center) <= X_range &&
                        (subSeedPoint.y+Y_center) >= 0 && (subSeedPoint.y+Y_center) <= Y_range &&
                        (subSeedPoint.z+Z_center) >= 0 && (subSeedPoint.z+Z_center) <= Z_range)
                    positionPoseScan(positionMatrix,subSeedPoint,subSeedOritationEuler,group);
            }

//    if(success)
//    {
//        std::cout<<"plan success"<<std::endl;
//        group.execute(my_plan);
//        std::cout<<my_plan.trajectory_.joint_trajectory.joint_names.back()<<std::endl;
//        std::cout<<my_plan.trajectory_.joint_trajectory.points.back()<<std::endl;
//    }
//    else
//    {
//        std::cout<<"plan faild"<<std::endl;
//    }
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "x_arm_test_random");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveItErrorCode success;

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::MoveGroupInterface group("x_arm");
  group.setPlanningTime(0.1);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  geometry_msgs::Pose ballPose;
  ballPose.orientation.w=1;
  ballPose.orientation.x=0;
  ballPose.orientation.y=0;
  ballPose.orientation.z=0;
  ballPose.position.x =1;
  ballPose.position.y =1;
  ballPose.position.z =1;

  addBall(planning_scene_interface,"base_link","ball1",ballPose,0.04,0);



  std::string fileName="poseSolve.YAML";
  cv::FileStorage fs(fileName, cv::FileStorage::WRITE);

  //generate poteitial oritation
  cv::Point3i seedPosition(0,0,66);
  cv::Point3d seedOritationEuler(-pi/2,pi/2,0);


  int sz[]={X_range,Y_range,Z_range,3};
  cv::Mat positionMatrix(4,sz,CV_64F,cv::Scalar(-100));

  positionPoseScan(positionMatrix,seedPosition,seedOritationEuler,group);


  fs<<"positionMatrix"<<positionMatrix;
  fs.release();

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");   

  //让机械臂按照规划的轨迹开始运动。
  if(!success)
      group.execute(my_plan);

  collisionCheck();

  ros::waitForShutdown();
  return 0;
}


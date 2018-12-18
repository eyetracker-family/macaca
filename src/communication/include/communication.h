#include <ros/ros.h> 
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PointStamped.h>
#include <serial/serial.h>  //ROS has built-in serial package

using namespace std;
using namespace cv;

typedef union theta
{
	unsigned char theta[2];
	short angle;
}theta_t;

serial::Serial ser_arm,ser_hand; //declare the serial object
//tf::TransformListener tf_lighthouse_eyetracker_listener;
//tf::StampedTransform lighthouse_eyetracker;
geometry_msgs::PointStamped ls_pos,robot_pos;
unsigned char ball_pos_data[16],hand_pos_data[16],hand_cmd[5];

void Ser_Arm_Initialize()
{
    try 
    { 
        ser_arm.setPort("/dev/ttyUSB0"); 
        ser_arm.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser_arm.setTimeout(to); 
        ser_arm.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port for Arm"); 
        return; 
    } 
    if(ser_arm.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial_Arm Port initialized"); 
    } 
    else 
    { 
        return; 
    } 
}

void Ser_Hand_Initialize()
{
    try 
    { 
        ser_hand.setPort("/dev/ttyUSB1"); 
        ser_hand.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser_hand.setTimeout(to); 
        ser_hand.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port for Hand"); 
        return; 
    } 
    if(ser_hand.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial_Hand Port initialized"); 
    } 
    else 
    { 
        return; 
    } 
}

void transformPoint(const tf::TransformListener &listener)
{
	geometry_msgs::PointStamped pos_target,robot_pos1;
	try
	{
		pos_target.header.frame_id="lscene_link";
		pos_target.header.stamp=ros::Time();
		pos_target.point.x=0;
		pos_target.point.y=0;
		pos_target.point.z=0;

		listener.transformPoint("robot_link",pos_target,robot_pos1);//coordinate transform robot_link
		listener.transformPoint("robot_link",ls_pos,robot_pos);//coordinate transform robot_link
	}
	catch(tf::TransformException &ex)
	{
		ROS_ERROR("exception aroused while coordinate transform!!!");
	}
	//cout<<"lscene in robot: "<<endl<<robot_pos1<<endl;
	cout<<"ball in robot: "<<endl<<robot_pos<<endl;
}

void pos_callback(const geometry_msgs::PointStamped::ConstPtr& msg) 
{
	ls_pos=*msg;
}

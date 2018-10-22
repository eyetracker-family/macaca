#include <ros/ros.h> 
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>

//#include "reconstructure/reconstructure.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <serial/serial.h>  //ROS has built-in serial package
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <sstream>

using namespace std;
using namespace cv;

serial::Serial ser_arm,ser_hand; //declare the serial object
//tf::TransformListener tf_lighthouse_eyetracker_listener;
tf::StampedTransform lighthouse_eyetracker;
geometry_msgs::PointStamped ls_pos,lighthouse_pos;
unsigned char data[16],hand_cmd[5];

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
	//Mat pos_scene=(Mat_<double>(4,1)<<pos.x,pos.y,pos.z,1);//left scene

	/*Mat pos_scene=(Mat_<double>(4,1)<<0,0,0,1);//left scene
	Mat transform_matrix=(Mat_<double>(4,4)<<-1,0,0,31.49,0,-0.9848,-0.1736,0.37,0,-0.1736,0.9848,137.68,0,0,0,1);//transform matrix between left scene camera and eyetracker.
	//Mat pos_eyetracker
	Mat pos_eyetracker=transform_matrix*pos_scene;
	cout<<"eyetracker_pos: "<<endl<<pos_eyetracker<<endl;*/

	/*geometry_msgs::PointStamped object_pos,lighthouse_pos;
	object_pos.header.frame_id="lscene_link";//eyetracker_link lscene_link
	object_pos.header.stamp=ros::Time();
	object_pos.point.x=pos.x/1000;
	object_pos.point.y=pos.y/1000;
	object_pos.point.z=pos.z/1000;*/


	try
	{
		listener.transformPoint("robot_link",ls_pos,robot_pos);//coordinate transform
		//cout<<"pos_lighthouse: ["<<lighthouse_pos.point.x<<","<<lighthouse_pos.point.y<<","<<lighthouse_pos.point.z<<"]"<<endl;
	}
	catch(tf::TransformException &ex)
	{
		ROS_ERROR("exception aroused while coordinate transform!!!");
	}

	cout<<"robot_pos: "<<endl<<robot_pos<<endl;

}

void pos_callback(const geometry_msgs::PointStamped::ConstPtr& msg) 
{
	ls_pos=*msg;
	/*pos.x=msg->x;
	pos.y=msg->y;
	pos.z=msg->z;*/
}

int main (int argc, char** argv) 
{ 
    ros::init(argc, argv, "serial_example_node"); 
    ros::NodeHandle nh; 
  
    //ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback); 
    ros::Subscriber pos_sub = nh.subscribe("scene/pos", 1000, pos_callback); 

    //ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000); 
  
	Ser_Arm_Initialize();
	Ser_Hand_Initialize();


    ros::Rate loop_rate(50); 
	tf::TransformListener tf_lighthouse_eyetracker_listener;

	ros::Timer timer=nh.createTimer(ros::Duration(1.0),boost::bind(&transformPoint,boost::ref(tf_lighthouse_eyetracker_listener)));


	data[0]=0x55;data[1]=0x01;data[2]=0x01;data[3]=0x0C;//header

	/*data[4]=(short int)(msg->x*10)>>8;data[5]=(short int)(msg->x*10)&0xff;//x*/

	data[4]=0x01;data[5]=0x01;//roll
	data[6]=0x01;data[7]=0x01;//pitch
	data[8]=0x01;data[9]=0x01;//yaw

	//short int a=(short int)(robot_pos.point.x*1000*10);//plus 10 to reserve one digit after the dot.
	short int a=1540;
	unsigned char* temp1=(unsigned char *)&a;
	data[10]=temp1[1];data[11]=temp1[0];//x

	//short int b=(short int)(robot_pos.point.y*1000*10);
	short int b=-630;
	unsigned char* temp2=(unsigned char *)&b;
	data[12]=temp2[1];data[13]=temp2[0];//y

	//short int c=(short int)(robot_pos.point.z*1000*10);
	short int c=1560;
	unsigned char* temp3=(unsigned char *)&c;
	data[14]=temp3[1];data[15]=temp3[0];//z

	/*short int pi=-8;
	unsigned char* temp=(unsigned char *)&pi;
	printf("%#x\n",temp[1]);
	printf("%#x\n",temp[0]);
	//data[16]=temp[1];data[17]=temp[0];//z
	short int* res=(short int*)temp;
	cout<<"res: "<<*res<<endl;*/

    ROS_INFO_STREAM("Writing to serial port: " <<ls_pos.point.x<<","<<ls_pos.point.y<<","<<ls_pos.point.z); 

    size_t i=ser_arm.write(data,16);
	cout<<"data sended to serial: "<<i<<endl;


	string arm_returned_cmd;
    while(ros::ok()) 
    { 
		/*arm_returned_cmd = ser_arm.read(16+1);
		if(arm_returned_cmd[0]==0x55)
		{
			hand_cmd[0]=0xAA;hand_cmd[1]=0x01;hand_cmd[2]=0x04;hand_cmd[3]=0x03;hand_cmd[4]=0x55;
			int i=ser_hand.write(hand_cmd,5);
			arm_returned_cmd="abc";
		}*/

        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 
} 

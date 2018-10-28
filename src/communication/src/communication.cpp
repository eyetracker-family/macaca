#include <ros/ros.h> 
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PointStamped.h>
#include <serial/serial.h>  //ROS has built-in serial package

using namespace std;
using namespace cv;

serial::Serial ser_arm,ser_hand; //declare the serial object
//tf::TransformListener tf_lighthouse_eyetracker_listener;
tf::StampedTransform lighthouse_eyetracker;
geometry_msgs::PointStamped ls_pos,robot_pos;
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
	cout<<"robot_pos: "<<endl<<robot_pos<<endl;
}

void pos_callback(const geometry_msgs::PointStamped::ConstPtr& msg) 
{
	ls_pos=*msg;
}

int main (int argc, char** argv) 
{ 
    ros::init(argc, argv, "serial_example_node"); 
    ros::NodeHandle nh; 

    ros::Subscriber pos_sub = nh.subscribe("scene/left/point", 1000, pos_callback); 

	//Ser_Arm_Initialize();
	Ser_Hand_Initialize();

    ros::Rate loop_rate(50); 

    ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); 
    ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); 
    ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); 
    ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); 
    ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); 
    ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); 
    ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); ros::spinOnce(); 

	tf::TransformListener tf_robot_lscene_listener;
	ros::Timer timer=nh.createTimer(ros::Duration(1.0),boost::bind(&transformPoint,boost::ref(tf_robot_lscene_listener)));

	data[0]=0x55;data[1]=0x01;data[2]=0x01;data[3]=0x0C;//header

	data[4]=0x00;data[5]=0x00;//roll
	data[6]=0xff;data[7]=0xf6;//pitch
	data[8]=0x00;data[9]=0x04;//yaw

	//data[4]=(short int)(msg->x*10)>>8;data[5]=(short int)(msg->x*10)&0xff;//x

	short int a=(short int)(robot_pos.point.x*1000*10);//plus 10 to reserve one digit after the dot.
	//short int a=1540;
	unsigned char* temp1=(unsigned char *)&a;//temp[1] is the higher 8 digits
	data[10]=temp1[1];data[11]=temp1[0];//x
	//data[10]=0x06;data[11]=0x04;//x

	short int b=(short int)(robot_pos.point.y*1000*10);
	//short int b=-630;
	unsigned char* temp2=(unsigned char *)&b;
	data[12]=temp2[1];data[13]=temp2[0];//y
	//data[12]=0xFD;data[13]=0x89;//y

	short int c=(short int)(robot_pos.point.z*1000*10);
	//short int c=1560;
	unsigned char* temp3=(unsigned char *)&c;
	data[14]=temp3[1];data[15]=temp3[0];//z
	//data[14]=0x06;data[15]=0x18;//z

	short int pi=-630;
	unsigned char* temp=(unsigned char *)&pi;
	//unsigned char temp[2];
	//temp[1]=(short int)(-630)>>8;//higher 8 digits
	//temp[0]=(short int)(-630)&0xff;

	printf("%#x\n",temp[1]);//higher 8 digits
	printf("%#x\n",temp[0]);

	short int* res=(short int*)temp;
	//short int res=(short int)(temp[1]<<8|temp[0]);
	cout<<"res: "<<*res<<endl;

    ROS_INFO_STREAM("Writing to serial port: " <<robot_pos.point.x<<","<<robot_pos.point.y<<","<<robot_pos.point.z); 
	
    //size_t i=ser_arm.write(data,16);
	//cout<<"data sended to serial: "<<i<<endl;

	//hand_cmd[0]=0xAA;hand_cmd[1]=0x01;hand_cmd[2]=0x04;hand_cmd[3]=0x03;hand_cmd[4]=0x55;
	//int i=ser_hand.write(hand_cmd,5);

	string arm_returned_cmd;
    while(ros::ok()) 
    { 
		/*arm_returned_cmd = ser_arm.read(16+1);
		if(arm_returned_cmd[0]==0x55)
		{
			usleep(5000000);
			hand_cmd[0]=0xAA;hand_cmd[1]=0x01;hand_cmd[2]=0x04;hand_cmd[3]=0x03;hand_cmd[4]=0x55;
			int i=ser_hand.write(hand_cmd,5);
			arm_returned_cmd="abc";
		}*/

        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 
} 

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

serial::Serial ser; //declare the serial object
//tf::TransformListener tf_lighthouse_eyetracker_listener;
tf::StampedTransform lighthouse_eyetracker;
Point3d pos;

/*void write_callback(const std_msgs::String::ConstPtr& msg) 
{ 
    ROS_INFO_STREAM("Writing to serial port: " <<msg->data); 
    size_t i=ser.write(msg->data);   //send data via serial port
	cout<<"data sended: "<<i<<endl;
} */

void transformPoint(const tf::TransformListener &listener)
{
	//Mat pos_scene=(Mat_<double>(4,1)<<pos.x,pos.y,pos.z,1);//left scene
	Mat pos_scene=(Mat_<double>(4,1)<<0,0,0,1);//left scene
	Mat transform_matrix=(Mat_<double>(4,4)<<-1,0,0,31.49,0,-0.9848,-0.1736,0.37,0,-0.1736,0.9848,137.68,0,0,0,1);//transform matrix between left scene camera and eyetracker.
	//Mat pos_eyetracker
	Mat pos_eyetracker=transform_matrix*pos_scene;
	cout<<"eyetracker_pos: "<<endl<<pos_eyetracker<<endl;

	geometry_msgs::PointStamped object_pos,lighthouse_pos;
	object_pos.header.frame_id="eyetracker_link";
	object_pos.header.stamp=ros::Time();
	object_pos.point.x=pos_eyetracker.at<double>(0,0)/1000;
	object_pos.point.y=pos_eyetracker.at<double>(1,0)/1000;
	object_pos.point.z=pos_eyetracker.at<double>(2,0)/1000;

	try
	{
		listener.transformPoint("lighthouse_link",object_pos,lighthouse_pos);//coordinate transform
		//cout<<"pos_lighthouse: ["<<lighthouse_pos.point.x<<","<<lighthouse_pos.point.y<<","<<lighthouse_pos.point.z<<"]"<<endl;
	}
	catch(tf::TransformException &ex)
	{
		ROS_ERROR("exception aroused while coordinate transform!!!");
	}

	cout<<"lighthouse_pos: "<<endl<<lighthouse_pos<<endl;

	char data[16];
	data[0]=0X55;data[1]=0X01;data[2]=0X01;data[3]=0X0C;//header

	/*data[4]=(short int)(msg->x*10)>>8;data[5]=(short int)(msg->x*10)&0xff;//x
	data[6]=(short int)(msg->y*10)>>8;data[7]=(short int)(msg->y*10)&0xff;//y
	data[8]=(short int)(msg->z*10)>>8;data[9]=(short int)(msg->z*10)&0xff;//z*/

	data[4]=0x01;data[5]=0x01;//roll
	data[6]=0x01;data[7]=0x01;//pitch
	data[8]=0x01;data[9]=0x01;//yaw

	short int a=(short int)(lighthouse_pos.point.x*10);
	//short int a=183;
	char* temp1=(char *)&a;
	data[10]=temp1[1];data[11]=temp1[0];//x

	short int b=(short int)(lighthouse_pos.point.y*10);
	//short int b=-1;
	char* temp2=(char *)&a;
	data[12]=temp2[1];data[13]=temp2[0];//y

	short int c=(short int)(lighthouse_pos.point.z*10);
	//short int c=-1;
	char* temp3=(char *)&a;
	data[14]=temp3[1];data[15]=temp3[0];//z

	short int pi=1234;
	char* temp=(char *)&pi;
	//data[16]=temp[1];data[17]=temp[0];//z
	short int* res=(short int*)temp;
	cout<<"res: "<<*res<<endl;

	//data[16]=((short int)(1234))>>8;data[17]=((short int)(1234))&0xff;//z
	//cout<<"data[16]: "<<data[16]<<"data[17]: "<<data[17]<<endl;

	/*printf("%#x\n",data[16]);
	printf("%#x\n",data[17]);*/

    ROS_INFO_STREAM("Writing to serial port: " <<pos.x<<","<<pos.y<<","<<pos.z); 
	//double x=msg->x;
	//ostringstream os;
	//os<<'#'<<observed_object[0]<<'*'<<observed_object[1]<<'*'<<observed_object[2];
	//os<<msg->x;
	//int i=ser.write(os.str());
    size_t i=ser.write(data);   
	cout<<"data sended to serial: "<<i<<endl;
}

void pos_callback(const geometry_msgs::Point::ConstPtr& msg) 
{
	pos.x=msg->x;
	pos.y=msg->y;
	pos.z=msg->z;

} 
int main (int argc, char** argv) 
{ 
    ros::init(argc, argv, "serial_example_node"); 
    ros::NodeHandle nh; 
  
    //ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback); 
    ros::Subscriber pos_sub = nh.subscribe("scene/pos", 1000, pos_callback); 

    //ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000); 
  
    try 
    { 
        ser.setPort("/dev/ttyUSB1"); 
        ser.setBaudrate(115200); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("Unable to open port "); 
        return -1; 
    } 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 
    ros::Rate loop_rate(50); 
	tf::TransformListener tf_lighthouse_eyetracker_listener;

	ros::Timer timer=nh.createTimer(ros::Duration(1.0),boost::bind(&transformPoint,boost::ref(tf_lighthouse_eyetracker_listener)));

    while(ros::ok()) 
    { 
  
        /*if(ser.available()){ 
            ROS_INFO_STREAM("Reading from serial port\n"); 
            std_msgs::String result; 
            result.data = ser.read(ser.available()); 
            ROS_INFO_STREAM("Read: " << result.data); 
            read_pub.publish(result); 
        } */
		//cout<<observed_object[0]<<endl;
		//ostringstream os;
		//os<<'#'<<observed_object[0]<<'*'<<observed_object[1]<<'*'<<observed_object[2];
		//os<<'#'<<0<<'*'<<1<<'*'<<2;
		//int i=ser.write(os.str());
		//cout<<"data sended: "<<i<<endl;

		/*try
		{
			tf_lighthouse_eyetracker_listener.lookupTransform("/lighthouse_link","/eyetracker_link",ros::Time(0),lighthouse_eyetracker);
			cout<<lighthouse_eyetracker.getOrigin()<<endl;
		}
		catch(tf::TransformException &ex)
		{
			//ROS_ERROR("s%",ex.what());
			continue;
		}*/

        ros::spinOnce(); 
        loop_rate.sleep(); 
  
    } 
} 

#include <ros/ros.h> 
#include <tf/transform_listener.h>

//#include "reconstructure/reconstructure.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <serial/serial.h>  //ROS has built-in serial package
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <sstream>
using namespace std;

serial::Serial ser; //declare the serial object
tf::TransformListener tf_lighthouse_eyetracker_listener;
tf::StampedTransform lighthouse_eyetracker;

/*void write_callback(const std_msgs::String::ConstPtr& msg) 
{ 
    ROS_INFO_STREAM("Writing to serial port: " <<msg->data); 
    size_t i=ser.write(msg->data);   //send data via serial port
	cout<<"data sended: "<<i<<endl;
} */

void pos_callback(const geometry_msgs::Point::ConstPtr& msg) 
{
	geometry_msgs::PointStamped object_pos,lighthouse_pos;
	object_pos.header.frame_id="eyetracker_link";
	object_pos.header.stamp=ros::Time();
	object_pos.point.x=msg->x;
	object_pos.point.y=msg->y;
	object_pos.point.z=msg->z;
	tf_lighthouse_eyetracker_listener.transformPoint("lighthouse_link",object_pos,lighthouse_pos);//coordinate transform

	char data[18];
	data[0]=0X55;data[1]=0X01;data[2]=0X01;data[3]=0X0C;//header
	data[4]=(int)(msg->x*10)>>8;data[5]=(int)(msg->x*10)&0xff;//x
	data[6]=(int)(msg->y*10)>>8;data[7]=(int)(msg->y*10)&0xff;//y
	data[8]=(int)(msg->z*10)>>8;data[9]=(int)(msg->z*10)&0xff;//z

	data[10]=0x01;data[11]=0x01;//roll
	data[12]=0x01;data[13]=0x01;//pitch
	data[14]=0x01;data[15]=0x01;//yaw

	data[16]=((short int)(1234))>>8;data[17]=((short int)(1234))&0xff;//z
	cout<<"data[16]: "<<data[16]<<"data[17]: "<<data[17]<<endl;

    ROS_INFO_STREAM("Writing to serial port: " <<msg->x<<","<<msg->y<<","<<msg->z); 
	//double x=msg->x;
	ostringstream os;
	//os<<'#'<<observed_object[0]<<'*'<<observed_object[1]<<'*'<<observed_object[2];
	os<<msg->x;
	//int i=ser.write(os.str());
    size_t i=ser.write(data);   
	cout<<"data sended to serial: "<<i<<endl;
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
        ser.setPort("/dev/ttyUSB0"); 
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
	//tf::TransformListener tf_lighthouse_eyetracker_listener;
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

		try
		{
			tf_lighthouse_eyetracker_listener.lookupTransform("/lighthouse_link","/eyetracker_link",ros::Time(0),lighthouse_eyetracker);
			cout<<lighthouse_eyetracker.getOrigin()<<endl;
		}
		catch(tf::TransformException &ex)
		{
			//ROS_ERROR("s%",ex.what());
			continue;
		}

        ros::spinOnce(); 
        loop_rate.sleep(); 
  
    } 
} 

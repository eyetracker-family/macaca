#include <ros/ros.h> 
//#include "reconstructure/reconstructure.h"
#include <geometry_msgs/Point.h>
#include <serial/serial.h>  //ROS已经内置了的串口包 
#include <std_msgs/String.h> 
#include <std_msgs/Empty.h> 
#include <sstream>
using namespace std;
serial::Serial ser; //声明串口对象 
  
//回调函数 
void write_callback(const std_msgs::String::ConstPtr& msg) 
{ 
    ROS_INFO_STREAM("Writing to serial port: " <<msg->data); 
    size_t i=ser.write(msg->data);   //发送串口数据 
	cout<<"data sended: "<<i<<endl;
} 

void pos_callback(const geometry_msgs::Point::ConstPtr& msg) 
{ 
    ROS_INFO_STREAM("Writing to serial port: " <<msg->x<<","<<msg->y<<","<<msg->z); 
	//double x=msg->x;
	ostringstream os;
	//os<<'#'<<observed_object[0]<<'*'<<observed_object[1]<<'*'<<observed_object[2];
	os<<msg->x;
	//int i=ser.write(os.str());
    size_t i=ser.write(os.str());   //发送串口数据 
	cout<<"data sended to serial: "<<i<<endl;
} 
  
int main (int argc, char** argv) 
{ 
    //初始化节点 
    ros::init(argc, argv, "serial_example_node"); 
    //声明节点句柄 
    ros::NodeHandle nh; 
  
    //订阅主题，并配置回调函doc
    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback); 
    ros::Subscriber pos_sub = nh.subscribe("scene/pos", 1000, pos_callback); 
    //发布主题 
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000); 
  
    try 
    { 
    //设置串口属性，并打开串口 
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
  
    //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    { 
        ROS_INFO_STREAM("Serial Port initialized"); 
    } 
    else 
    { 
        return -1; 
    } 
  
    //指定循环的频率 
    ros::Rate loop_rate(50); 
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

        //处理ROS的信息，比如订阅消息,并调用回调函数 
        ros::spinOnce(); 
        loop_rate.sleep(); 
  
    } 
} 

#include "matrix_multiply.h"
#include <ros/ros.h> 
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <serial/serial.h>  //ROS has built-in serial package

typedef union theta
{
	unsigned char theta[2];
	short angle;
}theta_t;

serial::Serial ser_arm,ser_hand; //declare the serial object
geometry_msgs::PointStamped ls_pos,robot_pos;
unsigned char joint_angle_data[18],hand_pos_data[18],hand_cmd[5];

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

void pos_callback(const geometry_msgs::PointStamped::ConstPtr& msg) 
{
	ls_pos=*msg;
}

void transformPoint(const tf::TransformListener &listener)
{
	try
	{
		listener.transformPoint("robot_link",ls_pos,robot_pos);//coordinate transform robot_link
	}
	catch(tf::TransformException &ex)
	{
		ROS_ERROR("exception aroused while coordinate transform!!!");
	}
	//cout<<"lscene in robot: "<<endl<<robot_pos1<<endl;
	cout<<"ball in robot: "<<endl<<robot_pos<<endl;
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "reverse_kinematics_node"); 
    ros::NodeHandle nh; 
    ros::Rate loop_rate(50); 

	Ser_Arm_Initialize();

    ros::Subscriber pos_sub = nh.subscribe("scene/left/point", 1000, pos_callback); 

	tf::TransformListener tf_robot_lscene_listener;
	ros::Timer timer=nh.createTimer(ros::Duration(1.0),boost::bind(&transformPoint,boost::ref(tf_robot_lscene_listener)));

	joint_angle_data[0]=0x55;joint_angle_data[1]=0x0b;joint_angle_data[2]=0x05;joint_angle_data[3]=0x0e;//ball pos data header

	if(argc==1)
	{
		while(robot_pos.point.x==0)
    		ros::spinOnce();//waiting until ball detected

		kinemat_envar_t knmt;
		kinemat_envar_t *p_knmt;
		p_knmt = &knmt;

		p_knmt->newPos[0]=robot_pos.point.x*1000;
		p_knmt->newPos[1]=robot_pos.point.y*1000;
		p_knmt->newPos[2]=robot_pos.point.z*1000;

		Optimal_algorithm(p_knmt);

		cout << "reverse angle = "\
			<< p_knmt->inverse_angle.theta1 << " " << p_knmt->inverse_angle.theta2 << " "\
			<< p_knmt->inverse_angle.theta3 << " " << p_knmt->inverse_angle.theta4 << " "\
			<< p_knmt->inverse_angle.theta5 << " " << p_knmt->inverse_angle.theta5 << " "\
			<< p_knmt->inverse_angle.theta7 << endl;

		short int theta=(short int)(p_knmt->inverse_angle.theta1*1000);//plus 1000 to reserve three digit after the dot.
		unsigned char* temp=(unsigned char *)&theta;//temp[1] is the higher 8 digits
		joint_angle_data[4]=temp[1];joint_angle_data[5]=temp[0];//x

		theta=(short int)(p_knmt->inverse_angle.theta2*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[6]=temp[1];joint_angle_data[7]=temp[0];

		theta=(short int)(p_knmt->inverse_angle.theta3*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[8]=temp[1];joint_angle_data[9]=temp[0];

		theta=(short int)(p_knmt->inverse_angle.theta4*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[10]=temp[1];joint_angle_data[11]=temp[0];

		theta=(short int)(p_knmt->inverse_angle.theta5*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[12]=temp[1];joint_angle_data[13]=temp[0];

		theta=(short int)(p_knmt->inverse_angle.theta6*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[14]=temp[1];joint_angle_data[15]=temp[0];

		theta=(short int)(p_knmt->inverse_angle.theta7*1000);
		temp=(unsigned char *)&theta;
		joint_angle_data[16]=temp[1];joint_angle_data[17]=temp[0];


		size_t i=ser_arm.write(joint_angle_data,18);
		cout<<"data sended to serial: "<<i<<endl;
    	//ROS_INFO_STREAM("Writing to serial port: " <<a/10<<","<<b/10<<","<<c/10);
	}

	else if(argc==2)
	{
		joint_angle_data[1]=0x04;
		size_t i=ser_arm.write(joint_angle_data,18);
		cout<<"arm reseted"<<endl;
	}

	string arm_returned_cmd;
    while(ros::ok()) 
    { 

		arm_returned_cmd = ser_arm.read(18+1);
		if(arm_returned_cmd[0]==0x55&&arm_returned_cmd[1]==0x01&&arm_returned_cmd[2]==0x02&&arm_returned_cmd[3]==0x0e)
		{

		cout<<"yes(arm_returned_cmd[0]==0x55&&arm_returned_cmd[1]==0x01&&arm_returned_cmd[2]==0x02&&arm_returned_cmd[3]==0x0e"<<endl;
			theta_t theta6,theta7;
			theta6.theta[0]=arm_returned_cmd[14];theta6.theta[1]=arm_returned_cmd[15];
			theta7.theta[0]=arm_returned_cmd[16];theta7.theta[1]=arm_returned_cmd[17];
			short angle6_=theta6.angle;
			short angle7_=theta7.angle;
			cout<<"angle6: "<<(float)angle6_/100.0<<endl;
			cout<<"angle7: "<<(float)angle7_/100.0<<endl;
			float angle6=((float)angle6_*3.1415926)/18000.0;
			float angle7=((float)angle7_*3.1415926)/18000.0;

			geometry_msgs::PointStamped pos_hand_tracker2,pos_hand_robot;

			pos_hand_tracker2.header.frame_id="tracker2_link";
			pos_hand_tracker2.header.stamp=ros::Time();
			pos_hand_tracker2.point.x=0.07*cos(angle7)*sin(angle6);
			pos_hand_tracker2.point.y=-0.099-0.07*cos(angle6)*cos(angle7);
			pos_hand_tracker2.point.z=0.052+0.07*sin(angle7);

			tf_robot_lscene_listener.transformPoint("robot_link",pos_hand_tracker2,pos_hand_robot);//coordinate transform robot_link

			cout<<"hand in robot: "<<endl<<pos_hand_robot<<endl;

			short int a=(short int)(pos_hand_robot.point.x*1000*10);//plus 10 to reserve one digit after the dot.
			//short int a=1540;
			unsigned char* temp1=(unsigned char *)&a;//temp[1] is the higher 8 digits
			hand_pos_data[10]=temp1[1];hand_pos_data[11]=temp1[0];//x

			short int b=(short int)(pos_hand_robot.point.y*1000*10);//plus 10 to reserve one digit after the dot.
			//short int a=1540;
			unsigned char* temp2=(unsigned char *)&b;//temp[1] is the higher 8 digits
			hand_pos_data[12]=temp2[1];hand_pos_data[13]=temp2[0];//x

			short int c=(short int)(pos_hand_robot.point.z*1000*10);//plus 10 to reserve one digit after the dot.
			//short int a=1540;
			unsigned char* temp3=(unsigned char *)&c;//temp[1] is the higher 8 digits
			hand_pos_data[14]=temp3[1];hand_pos_data[15]=temp3[0];//x

			//size_t i=ser_arm.write(hand_pos_data,16);
			//cout<<"data sended to serial: "<<i<<endl;
			//ROS_INFO_STREAM("Writing to serial port: " <<a/10<<","<<b/10<<","<<c/10);
		}

        ros::spinOnce(); 
        loop_rate.sleep(); 
    } 
}

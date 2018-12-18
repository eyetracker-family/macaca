#include "../include/communication.h"

int main (int argc, char** argv) 
{ 
    ros::init(argc, argv, "serial_example_node"); 
    ros::NodeHandle nh; 

    ros::Subscriber pos_sub = nh.subscribe("scene/left/point", 1000, pos_callback); 

	cout<<"num of argc: "<<argc<<endl;

	Ser_Arm_Initialize();
	//Ser_Hand_Initialize();

    ros::Rate loop_rate(50); 

	tf::TransformListener tf_robot_lscene_listener;
	ros::Timer timer=nh.createTimer(ros::Duration(1.0),boost::bind(&transformPoint,boost::ref(tf_robot_lscene_listener)));

	ball_pos_data[0]=0x55;ball_pos_data[1]=0x01;ball_pos_data[2]=0x01;ball_pos_data[3]=0x0C;//ball pos data header

	ball_pos_data[4]=0x00;ball_pos_data[5]=0x00;//roll
	ball_pos_data[6]=0xff;ball_pos_data[7]=0xf6;//pitch
	ball_pos_data[8]=0x00;ball_pos_data[9]=0x04;//yaw

	hand_pos_data[0]=0x55;hand_pos_data[1]=0x0a;hand_pos_data[2]=0x01;hand_pos_data[3]=0x0C;//hand pos data header

	hand_pos_data[4]=0x00;hand_pos_data[5]=0x00;//roll
	hand_pos_data[6]=0xff;hand_pos_data[7]=0xf6;//pitch
	hand_pos_data[8]=0x00;hand_pos_data[9]=0x04;//yaw

	if(argc==1)
	{
		while(robot_pos.point.x==0)
    		ros::spinOnce();

		short int a=(short int)(robot_pos.point.x*1000*10);//plus 10 to reserve one digit after the dot.
		//short int a=1540;
		unsigned char* temp1=(unsigned char *)&a;//temp[1] is the higher 8 digits
		ball_pos_data[10]=temp1[1];ball_pos_data[11]=temp1[0];//x

		short int b=(short int)(robot_pos.point.y*1000*10);
		//short int b=-630;
		unsigned char* temp2=(unsigned char *)&b;
		ball_pos_data[12]=temp2[1];ball_pos_data[13]=temp2[0];//y

		short int c=(short int)(robot_pos.point.z*1000*10);
		//short int c=1560;
		unsigned char* temp3=(unsigned char *)&c;
		ball_pos_data[14]=temp3[1];ball_pos_data[15]=temp3[0];//z

		size_t i=ser_arm.write(ball_pos_data,16);
		cout<<"data sended to serial: "<<i<<endl;
    	ROS_INFO_STREAM("Writing to serial port: " <<a/10<<","<<b/10<<","<<c/10);
	}

	else if(argc==2)
	{
		ball_pos_data[1]=0x04;
		size_t i=ser_arm.write(ball_pos_data,16);
		cout<<"arm reseted"<<endl;
	}

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

		arm_returned_cmd = ser_arm.read(18+1);
		if(arm_returned_cmd[0]==0x55&&arm_returned_cmd[1]==0x01&&arm_returned_cmd[2]==0x02&&arm_returned_cmd[3]==0x0e)
		{
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

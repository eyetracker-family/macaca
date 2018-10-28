#include "LighthouseTracking.h"
#include <tf/transform_broadcaster.h>
#include <iostream>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

void printTF(tf::Transform& t)
{
	tf::Matrix3x3& m = t.getBasis();
	tf::Vector3& v = t.getOrigin();
	for(int i = 0; i < 3; i++)
		printf("%3.4f %3.4f %3.4f\n", m[i].x(), m[i].y(), m[i].z());
	printf("%3.4f %3.4f %3.4f\n", v.x(), v.y(), v.z());
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"vive_tracker");
	ros::NodeHandle nh("~");

    InitFlags flags;
    flags.pipeCoords =false;
    if(flags.pipeCoords)
    {
        flags.printCoords = false;
        flags.printAnalog = false;
        flags.printEvents = false;
        flags.printSetIds = false;
        flags.printBEvents = false;
        flags.printTrack = false;
        flags.printRotation = false;
    }
    LighthouseTracking* htcViveTracker = new LighthouseTracking(flags);
    if (htcViveTracker) //null check
    {
        cout<<"htc vive initialized successfully"<<endl;
        htcViveTracker->iterateAssignIds();
    }
	tf::TransformBroadcaster *tf_broadcaster = new tf::TransformBroadcaster();
	ros::Rate rate(30);
	while(nh.ok())
	{	
		//cpSleep(50);
    	double pose_tracker1[7]={0,0,0,0,0,0,0};
		double r=-0.0455554277;
		double p=0.327044964;
    	htcViveTracker->getPoseMatrix(3, 0, pose_tracker1);

		tf::Quaternion rotation(pose_tracker1[1], pose_tracker1[2],pose_tracker1[3],pose_tracker1[0]);//Quaternion initialization:x,y,z,w  imaginary part followed by real part
		tf::Vector3 origin(pose_tracker1[4], pose_tracker1[5], pose_tracker1[6]);
		if (!isnan(rotation.x()) && !isnan(rotation.y())&& !isnan(rotation.z()) && !isnan(rotation.w()) && std::abs(origin.x()) < 5 && std::abs(origin.y()) < 5 && abs(origin.z()) < 5)
		{
			tf::Transform t(rotation, origin);
			/*
			Mat transform_pitch=(Mat_<double>(4,4)<<1,0,0,0,0,cos(-p),sin(p),0,0,-sin(p),cos(p),0,0,0,0,1);
			Mat transform_roll=(Mat_<double>(4,4)<<cos(r),sin(r),0,0,-sin(r),cos(r),0,0,0,0,1,0,0,0,0,1);
			cv::Mat mat4d=transform_pitch*transform_roll;
			tf::Transform merged_t;

			tf::Vector3 origin;
			tf::Matrix3x3 tf3d;

			origin.setValue (mat4d.at<double>(0, 3),
				             mat4d.at<double>(1, 3),
				             mat4d.at<double>(2, 3)
				            );
			tf3d.setValue(
			  mat4d.at<double>(0, 0), mat4d.at<double>(0, 1), mat4d.at<double>(0, 2),
			  mat4d.at<double>(1, 0), mat4d.at<double>(1, 1), mat4d.at<double>(1, 2),
			  mat4d.at<double>(2, 0), mat4d.at<double>(2, 1), mat4d.at<double>(2, 2)
			  );
			tf::Quaternion rotation;
			tf3d.getRotation(rotation);
			merged_t.setOrigin(origin);
			merged_t.setRotation(rotation);
			t=merged_t*t;
*/
			

			tf::Transform rx(tf::Quaternion(0,p,r),tf::Vector3(0,0,0));
			//printTF(rx);
			t=rx*t;
			tf::StampedTransform Tracker1ToLighthouse(t, ros::Time::now(),"lighthouse_link","tracker1_link");
			tf_broadcaster->sendTransform(Tracker1ToLighthouse);
		}

    	/*double pose_tracker2[6]={0,0,0,0,0,0};
    	htcViveTracker->getPoseMatrix(3, 1, pose_tracker2);

		tf::Quaternion rotation2(pose_tracker2[0], pose_tracker2[1], pose_tracker2[2]);
		tf::Vector3 origin2(pose_tracker2[3], pose_tracker2[4], pose_tracker2[5]);

		if (!isnan(rotation2.x()) && !isnan(rotation2.y())&& !isnan(rotation2.z()) && !isnan(rotation2.w()) && std::abs(origin2.x()) < 5 && std::abs(origin2.y()) < 5 && abs(origin2.z()) < 5 ) {
		  tf::Transform t2(rotation2, origin2);
			tf::Transform rx2(tf::Quaternion(0,p,r),tf::Vector3(0,0,0));
			t2=rx2*t2;
		  tf::StampedTransform Tracker2ToLighthouse(t2, ros::Time::now(),"lighthouse_link","tracker2_link");
		  tf_broadcaster->sendTransform(Tracker2ToLighthouse);
		}*/

		cout<<1<<"rotation: "<<pose_tracker1[0]<<"\t"<<pose_tracker1[1]<<"\t"<<pose_tracker1[2]<<"\t"<<pose_tracker1[3]<<"\t"<<"translation: "<<pose_tracker1[4]<<"\t"<<pose_tracker1[5]<<"\t"<<pose_tracker1[6]<<endl;
		//cout<<2<<"rotation: "<<pose_tracker2[0]<<"\t"<<pose_tracker2[1]<<"\t"<<pose_tracker2[2]<<"\t"<<pose_tracker2[3]<<"\t"<<"translation: "<<pose_tracker2[4]<<"\t"<<pose_tracker2[5]<<"\t"<<pose_tracker2[6]<<endl;

		rate.sleep();
	}
}

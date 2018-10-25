#include "LighthouseTracking.h"
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <ros/ros.h>

using namespace std;

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
    	htcViveTracker->getPoseMatrix(3, 0, pose_tracker1);

		tf::Quaternion rotation(pose_tracker1[1], pose_tracker1[2],pose_tracker1[3],pose_tracker1[0]);//Quaternion initialization:x,y,z,w  imaginary part followed by real part
		tf::Vector3 origin(pose_tracker1[4], pose_tracker1[5], pose_tracker1[6]);
		if (!isnan(rotation.x()) && !isnan(rotation.y())&& !isnan(rotation.z()) && !isnan(rotation.w()) && std::abs(origin.x()) < 5 && std::abs(origin.y()) < 5 && abs(origin.z()) < 5 ) 
		{
			tf::Transform t(rotation, origin);
			tf::StampedTransform Tracker1ToLighthouse(t, ros::Time::now(),"lighthouse_link","tracker1_link");
			tf_broadcaster->sendTransform(Tracker1ToLighthouse);
		}

    	double pose_tracker2[6]={0,0,0,0,0,0};
    	htcViveTracker->getPoseMatrix(3, 1, pose_tracker2);

		tf::Quaternion rotation2(pose_tracker2[0], pose_tracker2[1], pose_tracker2[2]);
		tf::Vector3 origin2(pose_tracker2[3], pose_tracker2[4], pose_tracker2[5]);

		if (!isnan(rotation2.x()) && !isnan(rotation2.y())&& !isnan(rotation2.z()) && !isnan(rotation2.w()) && std::abs(origin2.x()) < 5 && std::abs(origin2.y()) < 5 && abs(origin2.z()) < 5 ) {
		  tf::Transform t2(rotation2, origin2);
		  tf::StampedTransform Tracker2ToLighthouse(t2, ros::Time::now(),"lighthouse_link","tracker2_link");
		  tf_broadcaster->sendTransform(Tracker2ToLighthouse);
		}

		cout<<1<<"rotation: "<<pose_tracker1[0]<<"\t"<<pose_tracker1[1]<<"\t"<<pose_tracker1[2]<<"\t"<<"translation: "<<pose_tracker1[4]<<"\t"<<pose_tracker1[5]<<"\t"<<pose_tracker1[6]<<endl;
		cout<<2<<"rotation: "<<pose_tracker2[0]<<"\t"<<pose_tracker2[1]<<"\t"<<pose_tracker2[2]<<"\t"<<"translation: "<<pose_tracker2[4]<<"\t"<<pose_tracker2[5]<<"\t"<<pose_tracker2[6]<<endl;

		rate.sleep();
	}
}

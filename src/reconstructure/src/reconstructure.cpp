#include "../include/reconstructure/reconstructure.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reconstructure");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub_left=it.subscribe("scene/left/image_raw",1,ImageCallback_left);
    image_transport::Subscriber sub_right=it.subscribe("scene/right/image_raw",1,ImageCallback_right);
	image_transport::Publisher left_pub= it.advertise("scene/left/image", 1);
	image_transport::Publisher right_pub= it.advertise("scene/right/image", 1);

	ros::Publisher pos_pub=nh.advertise<geometry_msgs::PointStamped>("scene/left/point",1000);
	ros::Subscriber sub=nh.subscribe("/scene/left/fit_point",1000,ImagePoint_callback);

	bm->setPreFilterCap(31);//31
	bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
	bm->setMinDisparity(0);
	bm->setTextureThreshold(10);//10
	bm->setUniquenessRatio(15);//15
	bm->setSpeckleWindowSize(100);
	bm->setSpeckleRange(32);
	bm->setDisp12MaxDiff(1);

	sgbm->setPreFilterCap(63);
	int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
	sgbm->setBlockSize(sgbmWinSize);
	sgbm->setMinDisparity(0);
	sgbm->setNumDisparities(numberOfDisparities);
	sgbm->setUniquenessRatio(10);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setSpeckleRange(32);
	sgbm->setDisp12MaxDiff(1);
	sgbm->setMode(StereoSGBM::MODE_SGBM);

	//namedWindow("disparity", 0);
	//setMouseCallback("disparity", onMouse, 0);

	stereo_calibrate_initialize();

	while(nh.ok())
    {
		ros::spinOnce();
		if(img1_raw.empty()||img2_raw.empty()) 
		{
			//cout<<"no image"<<endl;
			continue;
		}

		Mat img1_remap,img2_remap,img1_remap_dst,img2_remap_dst;//undistorted grey image for reconstructure

		remap(img1_raw, img1_remap, map11, map12, INTER_LINEAR);
		remap(img2_raw, img2_remap, map21, map22, INTER_LINEAR);

	    cv:: cvtColor(img1_remap,img1_remap_dst,cv::COLOR_BGR2GRAY);
	    cv:: cvtColor(img2_remap,img2_remap_dst,cv::COLOR_BGR2GRAY);

		//multi object
		vector<std::pair<int,long long>> center_l,center_r;//label and size

		int64 t=getTickCount();

		Mat grey0,bin0,labels0,stats0,centroids0,grey1,bin1,labels1,stats1,centroids1;

		cv::GaussianBlur(img1_remap_dst,grey0,Size(5,5),0,0);
		cv::GaussianBlur(img2_remap_dst,grey1,Size(5,5),0,0);

		adaptiveThreshold(grey0,bin0,255,cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY,601,-60);
		adaptiveThreshold(grey1,bin1,255,cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY,601,-60);

		int nccomps0,nccomps1;
		nccomps0=cv::connectedComponentsWithStats(bin0,labels0,stats0,centroids0);
		nccomps1=cv::connectedComponentsWithStats(bin1,labels1,stats1,centroids1);

		t=getTickCount()-t;
		printf("Time elapsed by GetMultiColorBlockCenter: %fms\n",t*1000/getTickFrequency());

		for(int i=1;i<stats0.rows;i++)//begin from 1 because the 0 is background
		{
			if(stats0.at<int>(i,4)>700&&stats0.at<int>(i,4)<140000)
			{
				center_l.push_back(pair<int,long long>(i,stats0.at<int>(i,4)));
				//cout<<"connected component coordinate: "<<Point(centroids0.at<double>(i,0),centroids0.at<double>(i,1))<<endl;
			}
		}

		for(int i=1;i<stats1.rows;i++)//begin from 1 because the first one is background
		{
			if(stats1.at<int>(i,4)>700&&stats1.at<int>(i,4)<140000)
			{
				center_r.push_back(pair<int,long long>(i,stats1.at<int>(i,4)));
			}
		}
		
		std::sort(center_l.begin(),center_l.end(),[](const std::pair<int,long long>&a,const std::pair<int,long long>&b){return a.second>b.second;});//sort from bigger to smaller
		std::sort(center_r.begin(),center_r.end(),[](const std::pair<int,long long>&a,const std::pair<int,long long>&b){return a.second>b.second;});

		imshow("left_binary", bin0);
		imshow("right_binary", bin1);

		for(int i=0;i<center_l.size();i++)//draw the centers
		{
			circle(img1_remap,Point(centroids0.at<double>(center_l[i].first,0),centroids0.at<double>(center_l[i].first,1)),5,Scalar(0,0,0),3,8,0);///!!!
			stringstream ss;
			ss<<i+1;
			string str=ss.str();
			putText(img1_remap,str,Point(centroids0.at<double>(center_l[i].first,0)+20,centroids0.at<double>(center_l[i].first,1)+20),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,0));
			putText(img1_remap,"target",Point(centroids0.at<double>(1,0)-45,centroids0.at<double>(1,1)-20),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,0));//num_target_object
		}
		circle(img1_remap,Point(gaze_point_array[29].x,gaze_point_array[29].y),5,Scalar(0,0,255),3,8,0);///!!!
		for(int i=0;i<center_r.size();i++)
		{
			circle(img2_remap,Point(centroids1.at<double>(center_r[i].first,0),centroids1.at<double>(center_r[i].first,1)),5,Scalar(0,0,0),3,8,0);//multi object///!!!
			stringstream ss;
			ss<<i+1;
			string str=ss.str();
			putText(img2_remap,str,Point(centroids1.at<double>(center_r[i].first,0)+20,centroids1.at<double>(center_r[i].first,1)+20),FONT_HERSHEY_SIMPLEX,1,Scalar(0,0,0));
		}

		imshow("remap_left", img1_remap);
		imshow("remap_right", img2_remap);

		sensor_msgs::ImagePtr left_pub_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8", img1_remap).toImageMsg();
		left_pub.publish(left_pub_msg);
		sensor_msgs::ImagePtr right_pub_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8", img2_remap).toImageMsg();
		right_pub.publish(right_pub_msg);

		numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & -16;

		bm->setROI1(roi1);
		bm->setROI2(roi2);
		bm->setNumDisparities(numberOfDisparities);

		int cn = img1_remap_dst.channels();

		sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
		sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);

		copyMakeBorder(img1_remap_dst, img1_remap_dst, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);//left border
		copyMakeBorder(img2_remap_dst, img2_remap_dst, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

		//bm->compute(img1_remap_dst, img2_remap_dst, disp);//comment it for real time
		//sgbm->compute(img1_remap_dst, img2_remap_dst, disp);

		//disp = disp.colRange(numberOfDisparities, img1_remap_dst.cols);
		//cout<<disp.size()<<endl;

		size_t minimum=min(center_l.size(),center_r.size());//multi object
		int count1=1;
		for(int i=0;i<minimum;i++)
		{
			if(centroids0.at<double>(center_l[i].first,0)-centroids1.at<double>(center_r[i].first,0)>0&&centroids0.at<double>(center_l[i].first,0)-centroids1.at<double>(center_r[i].first,0)<300&&minimum<5)
//&&(center_l[i].second/center_r[i].second>0.6)&&(center_l[i].second/center_r[i].second<1.5)
			{
				disp.at<short>(centroids0.at<double>(center_l[i].first,1),centroids0.at<double>(center_l[i].first,0))=(short)(16*(centroids0.at<double>(center_l[i].first,0)-centroids1.at<double>(center_r[i].first,0)));//be careful of the order of x,y!!!///!!!
				cout<<"current pos of "<<count1<<" object in left image coordinate:"<<Point(centroids0.at<double>(center_l[i].first,0),centroids0.at<double>(center_l[i].first,1))<<endl;
				cout<<"current diaparity of "<<count1<<" ball center:"<<centroids0.at<double>(center_l[i].first,0)-centroids1.at<double>(center_r[i].first,0)<<endl;
				count1++;
			}
		}
		cout<<"there is totally "<<count1-1<<" object"<<endl;

		//disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));

		reprojectImageTo3D(disp, xyz, Q, true);
		xyz = xyz * 16;

		//decide the observing object by the cloest center
		/*int count2=1;
		vector<pair<Point2d,Point3d>> center_position_array;
		for(int i=0;i<minimum;i++)
		{
			if(centroids0.at<double>(center_l[i].first,0)-centroids1.at<double>(center_r[i].first,0)>0&&centroids0.at<double>(center_l[i].first,0)-centroids1.at<double>(center_r[i].first,0)<300&&minimum<5)//multi object
			{
				cout<<"The "<<count2<<"th object is at ["<< xyz.at<Vec3f>(centroids0.at<double>(center_l[i].first,1),centroids0.at<double>(center_l[i].first,0))[0]<<","<<-xyz.at<Vec3f>(centroids0.at<double>(center_l[i].first,1),centroids0.at<double>(center_l[i].first,0))[1]<<","<<xyz.at<Vec3f>(centroids0.at<double>(center_l[i].first,1),centroids0.at<double>(center_l[i].first,0))[2]<<"]"<<endl;
				count2++;

				center_position_array.push_back(pair<Point2d,Point3d>(Point2d(centroids0.at<double>(center_l[i].first,0),centroids0.at<double>(center_l[i].first,1)),Point3d(xyz.at<Vec3f>(centroids0.at<double>(center_l[i].first,1),centroids0.at<double>(center_l[i].first,0))[0],-xyz.at<Vec3f>(centroids0.at<double>(center_l[i].first,1),centroids0.at<double>(center_l[i].first,0))[1],xyz.at<Vec3f>(centroids0.at<double>(center_l[i].first,1),centroids0.at<double>(center_l[i].first,0))[2])));
			}
		}
		if(center_position_array.size()>0)
		{
			geometry_msgs::Point pos=FindClosestObject(center_position_array,gaze_point);
			cout<<"gaze_point: "<<gaze_point.x<<","<<gaze_point.y<<endl;
			cout<<"pos of observing object: "<<pos.x<<","<<pos.y<<","<<pos.z<<endl;

			pos_pub.publish(pos);//publish the positon of observed object.
		}*/

		//decide the observing object by the connected component
		int count2=1;
		vector<pair<int,Point3d>> label_position_array;
		for(int i=0;i<minimum;i++)
		{
			if(centroids0.at<double>(center_l[i].first,0)-centroids1.at<double>(center_r[i].first,0)>0&&centroids0.at<double>(center_l[i].first,0)-centroids1.at<double>(center_r[i].first,0)<300&&minimum<5)//multi object
			{
				cout<<"The "<<count2<<"th object is at ["<< xyz.at<Vec3f>(centroids0.at<double>(center_l[i].first,1),centroids0.at<double>(center_l[i].first,0))[0]<<","<<xyz.at<Vec3f>(centroids0.at<double>(center_l[i].first,1),centroids0.at<double>(center_l[i].first,0))[1]<<","<<xyz.at<Vec3f>(centroids0.at<double>(center_l[i].first,1),centroids0.at<double>(center_l[i].first,0))[2]<<"]"<<endl;
				count2++;

				label_position_array.push_back(pair<int,Point3d>(center_l[i].first,Point3d(xyz.at<Vec3f>(centroids0.at<double>(center_l[i].first,1),centroids0.at<double>(center_l[i].first,0))[0],xyz.at<Vec3f>(centroids0.at<double>(center_l[i].first,1),centroids0.at<double>(center_l[i].first,0))[1],xyz.at<Vec3f>(centroids0.at<double>(center_l[i].first,1),centroids0.at<double>(center_l[i].first,0))[2])));
			}
		}
		geometry_msgs::PointStamped pos;
		if(label_position_array.size()>0)
		{
			pos=Find_Target_Object(label_position_array,gaze_point_array,labels0,nccomps0,num_target_object);
		}
		else
			pos.point.x=pos.point.y=pos.point.z=0;

		cout<<"gaze_point: "<<gaze_point_array[29].x<<","<<gaze_point_array[29].y<<endl;
		cout<<"pos of observing object: "<<pos.point.x<<","<<pos.point.y<<","<<pos.point.z<<endl;

		if(pos.point.x!=0)
			pos_pub.publish(pos);//publish the positon of observed object.

		//Mat vdispRGB = disp8;
		//cvtColor(disp8, vdispRGB, COLOR_GRAY2BGR);

		/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloud=MatToPoinXYZ(xyz);	

		if(cloud!=nullptr)
		{
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			sor.setInputCloud (cloud);
			sor.setMeanK (50);
			sor.setStddevMulThresh (1.0);
			sor.filter (*cloud);
		}

		viewer.showCloud(cloud);  */

    	//viewer.runOnVisualizationThreadOnce (viewerOneOff);

		//imshow("disparity", disp8);


		//wait for 40 milliseconds
		int c = waitKey(1);

		//exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)
		if (27 == char(c)) break;
    }
    return 0;
   //cv::destroyWindow("view");
 }

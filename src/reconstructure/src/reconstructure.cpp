#include "../include/reconstructure/reconstructure.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
   //cv::namedWindow("view");
	image_transport::ImageTransport it_left(nh);

    image_transport::ImageTransport it_right(nh);

    image_transport::Subscriber sub_left=it_left.subscribe("scene/left/image_raw",1,ImageCallback_left);
    image_transport::Subscriber sub_right=it_right.subscribe("scene/right/image_raw",1,ImageCallback_right);

	ros::Publisher pos_pub=nh.advertise<geometry_msgs::Point>("scene/pos",1000);
	ros::Subscriber sub=nh.subscribe("/scene/left/fit_point",1000,ImagePoint_callback);


	FileStorage fs(intrinsic_filename, FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", intrinsic_filename.c_str());
		return -1;
	}

	Mat M1, D1, M2, D2;
	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;

	M1 *= scale;
	M2 *= scale;

	fs.open(extrinsic_filename, FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", extrinsic_filename.c_str());
		return -1;
	}

	Mat R, T, R1, P1, R2, P2;
	fs["R"] >> R;
	fs["T"] >> T;

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

	Size img_size(1280,720);
	//Size img_size=img1_raw.size();

	stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

	cout<<"roi1.size: "<<roi1.size()<<endl;
	cout<<"roi2.size: "<<roi2.size()<<endl;
	cout<<"matrix M1: "<<M1<<endl;
	cout<<"matrix D1: "<<D1<<endl;
	cout<<"matrix R1: "<<R1<<endl;
	cout<<"matrix R2: "<<R2<<endl;
	cout<<"matrix Q: "<<Q<<endl;

	Mat map11, map12, map21, map22;
	initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
	initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

	while(nh.ok())
    {
		ros::spinOnce();
		if(img1_raw.empty()||img2_raw.empty()) 
		{
			//cout<<"no image"<<endl;
			continue;
		}

		cv::Mat img1=img1_raw,img2=img2_raw;
	    cv:: cvtColor(img1_raw,img1,cv::COLOR_BGR2GRAY);
	    cv:: cvtColor(img2_raw,img2,cv::COLOR_BGR2GRAY);

	    //cv::imshow("left_grey",img1);

		//cout<<img1.size()<<endl;

		Mat img1r, img2r;//undistorted grey image for reconstructure
		remap(img1, img1r, map11, map12, INTER_LINEAR);
		remap(img2, img2r, map21, map22, INTER_LINEAR);

		//imshow("remap_left_grey", img1r);
		//imshow("remap_right_grey", img2r);

		img1 = img1r;
		img2 = img2r;
		
		/*//one object
		vector<Vec3f> circles_l;
		Point center_l,center_r;
		vector<Vec3f> circles_r;

		Mat img1r_raw, img2r_raw;
		remap(img1_raw, img1r_raw, map11, map12, INTER_LINEAR);
		remap(img2_raw, img2r_raw, map21, map22, INTER_LINEAR);

		center_l=GetColorBlockCenter(img1r_raw, binary_left, thresh, min_area, max_area);
		center_r=GetColorBlockCenter(img2r_raw, binary_right, thresh, min_area, max_area);//one object*/

		//multi object
		vector<std::pair<cv::Point,long long>> center_l,center_r;

		Mat img1r_raw, img2r_raw;//undistorted color image for connected component analysis.
		remap(img1_raw, img1r_raw, map11, map12, INTER_LINEAR);
		remap(img2_raw, img2r_raw, map21, map22, INTER_LINEAR);

		center_l=GetMultiColorBlockCenter(img1r_raw, binary_left, thresh, min_area, max_area);
		center_r=GetMultiColorBlockCenter(img2r_raw, binary_right, thresh, min_area, max_area);
		
		std::sort(center_l.begin(),center_l.end(),[](const std::pair<cv::Point,long long>&a,const std::pair<cv::Point,long long>&b){return a.second>b.second;});
		std::sort(center_r.begin(),center_r.end(),[](const std::pair<cv::Point,long long>&a,const std::pair<cv::Point,long long>&b){return a.second>b.second;});

		imshow("left_binary", binary_left);
		imshow("right_binary", binary_right);
		for(int i=0;i<center_l.size();i++)
			circle(img1r,center_l[i].first,5,Scalar(0,0,0),3,8,0);
		for(int i=0;i<center_r.size();i++)
			circle(img2r,center_r[i].first,5,Scalar(0,0,0),3,8,0);//multi object

		//hough circles
		/*static int frame_count=0;
		if(frame_count%10==0)
		{
			GaussianBlur(img1r,img1r,Size(9,9),2,2);
			HoughCircles(img1r,circles_l,CV_HOUGH_GRADIENT,1.5,10,200,100,30,80);

			GaussianBlur(img2r,img2r,Size(9,9),2,2);
			HoughCircles(img2r,circles_r,CV_HOUGH_GRADIENT,1.5,10,200,100,30,80);
			if(circles_r.size()>0&&circles_l.size()>0)
			{
				center_l=Point(circles_l[0][0],circles_l[0][1]);
				int radius_l=circles_l[0][2];
				circle(img1r,center_l,radius_l,Scalar(255,0,0),3,8,0);
				center_r=Point(circles_r[0][0],circles_r[0][1]);
				int radius_r=circles_r[0][2];
				circle(img2r,center_r,radius_r,Scalar(255,0,0),3,8,0);
			}
			if(frame_count>500) frame_count=0;
		}
		frame_count++;*/

		imshow("remap_left_grey", img1r);
		imshow("remap_right_grey", img2r);

		numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & -16;

		bm->setROI1(roi1);
		bm->setROI2(roi2);
		bm->setNumDisparities(numberOfDisparities);

		int cn = img1.channels();

		sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
		sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);

		copyMakeBorder(img1, img1, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
		copyMakeBorder(img2, img2, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

		//bm->compute(img1, img2, disp);//comment it for real time
		//sgbm->compute(img1, img2, disp);

		//disp = disp.colRange(numberOfDisparities, img1.cols);
		//cout<<disp.size()<<endl;

		//if(circles_l.size()>0&&circles_r.size()>0)
		//{
		//}

		/*if(center_l.x-center_r.x>0&&center_l.x-center_r.x<300)//one object
		{
			disp.at<short>(center_l.y,center_l.x)=16*(center_l.x-center_r.x);//be careful of the order of x,y!!!
			cout<<"current pos in left image coordinate:"<<center_l<<endl;
			cout<<"current diaparity of ball center:"<<center_l.x-center_r.x<<endl;
		}*/

		size_t minimum=min(center_l.size(),center_r.size());//multi object
		int count1=1;
		for(int i=0;i<minimum;i++)
		{
			if(center_l[i].first.x-center_r[i].first.x>0&&center_l[i].first.x-center_r[i].first.x<300&&minimum<5)
//&&(center_l[i].second/center_r[i].second>0.6)&&(center_l[i].second/center_r[i].second<1.5)
			{
				disp.at<short>(center_l[i].first.y,center_l[i].first.x)=16*(center_l[i].first.x-center_r[i].first.x);//be careful of the order of x,y!!!
				cout<<"current pos of "<<count1<<" object in left image coordinate:"<<center_l[i].first<<endl;
				cout<<"current diaparity of "<<count1<<" ball center:"<<center_l[i].first.x-center_r[i].first.x<<endl;
				count1++;
			}
		}
		cout<<"there is totally "<<count1-1<<" object"<<endl;

		//disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));

		reprojectImageTo3D(disp, xyz, Q, true);
		xyz = xyz * 16;
		//if(circles_l.size()>0&&circles_r.size()>0)
		//{
			//cout<<"The ball is at "<<xyz.at<Vec3f>(center_l.x,center_l.y)<<endl;
		//}

		/*if(center_l.x-center_r.x>0&&center_l.x-center_r.x<300)
		{
			cout<<"The ball is at ["<< xyz.at<Vec3f>(center_l.y,center_l.x)[0]<<","<<-xyz.at<Vec3f>(center_l.y,center_l.x)[1]<<","<<xyz.at<Vec3f>(center_l.y,center_l.x)[2]<<"]"<<endl;
		}
		else cout<<"there is no object"<<endl;*/
		int count2=1;
		vector<pair<Point2d,Point3d>> center_position_array;
		for(int i=0;i<minimum;i++)
		{
			if(center_l[i].first.x-center_r[i].first.x>0&&center_l[i].first.x-center_r[i].first.x<300&&minimum<5)//multi object
			{
				cout<<"The "<<count2<<"th object is at ["<< xyz.at<Vec3f>(center_l[i].first.y,center_l[i].first.x)[0]<<","<<-xyz.at<Vec3f>(center_l[i].first.y,center_l[i].first.x)[1]<<","<<xyz.at<Vec3f>(center_l[i].first.y,center_l[i].first.x)[2]<<"]"<<endl;
				count2++;

				center_position_array.push_back(pair<Point2d,Point3d>(Point2d(center_l[i].first.x,center_l[i].first.y),Point3d(xyz.at<Vec3f>(center_l[i].first.y,center_l[i].first.x)[0],-xyz.at<Vec3f>(center_l[i].first.y,center_l[i].first.x)[1],xyz.at<Vec3f>(center_l[i].first.y,center_l[i].first.x)[2])));
				
				//geometry_msgs::Point pos;//positon of observed object.
				//pos.x=xyz.at<Vec3f>(center_l[0].first.y,center_l[0].first.x)[0];
				//pos.y=-xyz.at<Vec3f>(center_l[0].first.y,center_l[0].first.x)[1];
				//pos.z=xyz.at<Vec3f>(center_l[0].first.y,center_l[0].first.x)[2];

				//observed_object[0]=xyz.at<Vec3f>(center_l[0].first.y,center_l[0].first.x)[0];
				//observed_object[1]=xyz.at<Vec3f>(center_l[0].first.y,center_l[0].first.x)[1];
				//observed_object[2]=xyz.at<Vec3f>(center_l[0].first.y,center_l[0].first.x)[2];
				//cout<<"observed_object[0]: "<<observed_object[0]<<endl;
			}
		}
		if(center_position_array.size()>0)
		{
			geometry_msgs::Point pos=FindClosestObject(center_position_array,gaze_point);
			cout<<"gaze_point: "<<gaze_point.x<<","<<gaze_point.y<<endl;
			cout<<"pos of observing object: "<<pos.x<<","<<pos.y<<","<<pos.z<<endl;

			pos_pub.publish(pos);//publish the positon of observed object.
		}

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
   //ros::spin();
   //cv::destroyWindow("view");
 }

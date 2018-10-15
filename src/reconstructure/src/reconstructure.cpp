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

		Mat img1_dst,img2_dst,img1r,img2r;//undistorted grey image for reconstructure
	    cv:: cvtColor(img1_raw,img1_dst,cv::COLOR_BGR2GRAY);
	    cv:: cvtColor(img2_raw,img2_dst,cv::COLOR_BGR2GRAY);

		//Mat img1r, img2r;//undistorted grey image for reconstructure
		remap(img1_dst, img1r, map11, map12, INTER_LINEAR);
		remap(img2_dst, img2r, map21, map22, INTER_LINEAR);

		//multi object
		vector<std::pair<cv::Point2d,long long>> center_l,center_r;//centroid and size

		int64 t=getTickCount();

		Mat grey0,dst0,labels0,stats0,centroids0,grey1,dst1,labels1,stats1,centroids1;

		cv::GaussianBlur(img1r,grey0,Size(5,5),0,0);
		cv::GaussianBlur(img2r,grey1,Size(5,5),0,0);

		adaptiveThreshold(grey0,dst0,255,cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY,601,-60);
		adaptiveThreshold(grey1,dst1,255,cv::ADAPTIVE_THRESH_MEAN_C,cv::THRESH_BINARY,601,-60);

		int nccomps0,nccomps1;
		nccomps0=cv::connectedComponentsWithStats(dst0,labels0,stats0,centroids0);
		nccomps1=cv::connectedComponentsWithStats(dst1,labels1,stats1,centroids1);

		for(int i=1;i<stats0.rows;i++)//begin from 1 because the first one is background
		{
			if(stats0.at<int>(i,4)>700&&stats0.at<int>(i,4)<70000)
			{
				center_l.push_back(pair<Point,long long>(Point(centroids0.at<double>(i,0),centroids0.at<double>(i,1)),stats0.at<int>(i,4)));
				//cout<<"connected component coordinate: "<<Point(centroids0.at<double>(i,0),centroids0.at<double>(i,1))<<endl;
			}
		}

		for(int i=1;i<stats1.rows;i++)//begin from 1 because the first one is background
		{
			if(stats1.at<int>(i,4)>700&&stats1.at<int>(i,4)<70000)
			{
				center_r.push_back(pair<Point,long long>(Point(centroids1.at<double>(i,0),centroids1.at<double>(i,1)),stats1.at<int>(i,4)));
			}
		}

		//center_l=GetMultiColorBlockCenter(img1r_raw, binary_left, thresh, min_area, max_area);
		//center_r=GetMultiColorBlockCenter(img2r_raw, binary_right, thresh, min_area, max_area);

		t=getTickCount()-t;
		printf("Time elapsed by GetMultiColorBlockCenter: %fms\n",t*1000/getTickFrequency());
		
		std::sort(center_l.begin(),center_l.end(),[](const std::pair<cv::Point,long long>&a,const std::pair<cv::Point,long long>&b){return a.second>b.second;});
		std::sort(center_r.begin(),center_r.end(),[](const std::pair<cv::Point,long long>&a,const std::pair<cv::Point,long long>&b){return a.second>b.second;});

		//imshow("left_binary", binary_left);
		//imshow("right_binary", binary_right);

		imshow("left_binary", dst0);
		imshow("right_binary", dst1);

		for(int i=0;i<center_l.size();i++)
			circle(img1r,center_l[i].first,5,Scalar(0,0,0),3,8,0);
		for(int i=0;i<center_r.size();i++)
			circle(img2r,center_r[i].first,5,Scalar(0,0,0),3,8,0);//multi object

		imshow("remap_left_grey", img1r);
		imshow("remap_right_grey", img2r);

		numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & -16;

		bm->setROI1(roi1);
		bm->setROI2(roi2);
		bm->setNumDisparities(numberOfDisparities);

		int cn = img1r.channels();

		sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
		sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);

		copyMakeBorder(img1r, img1r, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);//left border
		copyMakeBorder(img2r, img2r, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

		//bm->compute(img1r, img2r, disp);//comment it for real time
		//sgbm->compute(img1r, img2r, disp);

		//disp = disp.colRange(numberOfDisparities, img1r.cols);
		//cout<<disp.size()<<endl;

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

		int count2=1;
		vector<pair<Point2d,Point3d>> center_position_array;
		for(int i=0;i<minimum;i++)
		{
			if(center_l[i].first.x-center_r[i].first.x>0&&center_l[i].first.x-center_r[i].first.x<300&&minimum<5)//multi object
			{
				cout<<"The "<<count2<<"th object is at ["<< xyz.at<Vec3f>(center_l[i].first.y,center_l[i].first.x)[0]<<","<<-xyz.at<Vec3f>(center_l[i].first.y,center_l[i].first.x)[1]<<","<<xyz.at<Vec3f>(center_l[i].first.y,center_l[i].first.x)[2]<<"]"<<endl;
				count2++;

				center_position_array.push_back(pair<Point2d,Point3d>(Point2d(center_l[i].first.x,center_l[i].first.y),Point3d(xyz.at<Vec3f>(center_l[i].first.y,center_l[i].first.x)[0],-xyz.at<Vec3f>(center_l[i].first.y,center_l[i].first.x)[1],xyz.at<Vec3f>(center_l[i].first.y,center_l[i].first.x)[2])));
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

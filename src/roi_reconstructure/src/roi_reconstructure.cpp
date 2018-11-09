#include "../include/roi_reconstructure/roi_reconstructure.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reconstructure");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub_left=it.subscribe("scene/left/image_raw",1,ImageCallback_left);
    image_transport::Subscriber sub_right=it.subscribe("scene/right/image_raw",1,ImageCallback_right);

	image_transport::Publisher left_pub= it.advertise("scene/left/image", 1);//for rviz UI
	image_transport::Publisher right_pub= it.advertise("scene/right/image", 1);

	image_transport::Publisher left_pub_remap= it.advertise("scene/left/image_remap", 1);//for yolo dectection

	ros::Publisher pos_pub=nh.advertise<geometry_msgs::PointStamped>("scene/left/point",1000);
	ros::Subscriber sub=nh.subscribe("/scene/left/fit_point",1000,ImagePoint_callback);//fit point from gaussian regression

	ros::Subscriber yolo_sub=nh.subscribe("/darknet_ros/bounding_boxes",1000,BoundingBox_callback);

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

	stereo_calibrate_initialize();

	while(nh.ok())
    {
		ros::spinOnce();
		if(img1_raw.empty()||img2_raw.empty()) 
		{
			//cout<<"no image"<<endl;
			continue;
		}
		//去掉非roi区域
		Mat img1_remap,img2_remap,img1_remap_dst,img2_remap_dst;

		remap(img1_raw, img1_remap, map11, map12, INTER_LINEAR);
		remap(img2_raw, img2_remap, map21, map22, INTER_LINEAR);

		sensor_msgs::ImagePtr left_pub_msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8", img1_remap).toImageMsg();
		left_pub_remap.publish(left_pub_msg);

	    cv:: cvtColor(img1_remap,img1_remap_dst,cv::COLOR_BGR2GRAY);
	    cv:: cvtColor(img2_remap,img2_remap_dst,cv::COLOR_BGR2GRAY);

		numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & -16;

		bm->setROI1(roi1);
		bm->setROI2(roi2);
		bm->setNumDisparities(numberOfDisparities);

		int cn = img1_remap_dst.channels();

		sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
		sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);

		//copyMakeBorder(img1_remap_dst, img1_remap_dst, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);//left border
		//copyMakeBorder(img2_remap_dst, img2_remap_dst, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
        //imshow("img1_remap_dst",img1_remap_dst);
        //imshow("img2_remap_dst", img2_remap_dst);
        //cout<<img1_remap_dst.cols<<endl;
		int64 t=getTickCount();
		bm->compute(img1_remap_dst, img2_remap_dst, disp);//comment it for real time
        //imshow("disp",disp);
		//sgbm->compute(img1_remap_dst, img2_remap_dst, disp);
		t=getTickCount()-t;
		printf("Time elapsed by bm: %fms\n",t*1000/getTickFrequency());

		//disp = disp.colRange(numberOfDisparities, img1_remap_dst.cols);

		disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));

		reprojectImageTo3D(disp, xyz, Q, true);
		xyz = xyz * 16;

		cout<<"tracking_box: "<<tracking_box<<endl;
		static bool tracking_init_flag=false;
		if(tracking_box.height!=0)
		{
			if(tracking_init_flag)
				cv_tracker->clear();
			cv_tracker = TrackerKCF::create();
			//if(tracking_init_flag)
			//{
				cout<<"tracking_box.height: "<<tracking_box.height<<endl;
				cv_tracker->init(img1_remap,tracking_box);
				tracking_init_flag=true;
			//}
		}
		if(tracking_init_flag)
		{
			cv_tracker->update(img1_remap,tracked_box);
			rectangle(img1_remap,tracked_box,Scalar(255,0,0),2,1);
			imshow("Tracking",img1_remap);
		}

		/*if(tracking_box.height!=0)
		{
			//if(tracking_init_flag)
			{
				dlib::cv_image<dlib::rgb_pixel> dlib_img_init(img1_remap); 
				dlib_tracker.start_track(dlib_img_init, openCVRectToDlib(tracking_box));
				tracking_init_flag=false;
			}
			dlib::cv_image<dlib::rgb_pixel> dlib_img(img1_remap); 
			double rate=dlib_tracker.update(dlib_img);
			cout<<"rate: "<<rate<<endl;
			tracked_box=dlibRectangleToOpenCV(dlib_tracker.get_position());
			rectangle(img1_remap,tracked_box,Scalar(255,0,0),2,1);
			imshow("Tracking",img1_remap);
		}*/

		geometry_msgs::PointStamped pos;
		long long sumx=0,sumy=0,sumz=0,count=0;
		if(detected_object_array.size()>0)
		{
			//cout<<"bounding_box: "<<detected_object_array[0].bounding_box<<endl;
			for(int y=0;y<disp8.rows;y++)
				for(int x=0;x<disp8.cols;x++)
				{
					if(tracked_box.contains(Point2i(x,y))&&xyz.at<Vec3f>(y,x)[2]<1000)
					{
						sumx+=xyz.at<Vec3f>(y,x)[0];
						sumy+=xyz.at<Vec3f>(y,x)[1];
						sumz+=xyz.at<Vec3f>(y,x)[2];
						count++;
						//cout<<"xyz: "<<xyz.at<Vec3f>(y,x)<<endl;
					}
						//disp8.at<char>(y,x)=0;
				}
		}

		if(count!=0)
		{
			pos.header.frame_id="lscene_link";
			pos.header.stamp=ros::Time();
			pos.point.x=(sumx/(count))/1000.;
			pos.point.y=(sumy/(count))/1000.;
			pos.point.z=(sumz/(count))/1000.+0.02;
			pos_pub.publish(pos);
			cout<<"pos: "<<endl<<pos<<endl;
		}

		if(detected_object_array.size()>0)
		{
			rectangle(disp8,tracked_box,255,3,8,0);
		}

		imshow("disparity", disp8);

		int c = waitKey(1);

		if (27 == char(c)) break;
	}
}

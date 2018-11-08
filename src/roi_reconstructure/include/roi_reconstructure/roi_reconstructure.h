#include <iostream>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <eyetracking_msgs/ImagePoint.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <image_transport/image_transport.h>

#include <opencv2/tracking.hpp>

//#include <pcl/visualization/cloud_viewer.h>   
//#include <pcl/io/io.h>  
//#include <pcl/io/pcd_io.h>

//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/statistical_outlier_removal.h>

using namespace cv;
using namespace std;

static vector<double> observed_object(3,0.0);//object position to send
int num_target_object=0;

bool selectObject;
Rect selection;
Point2d origin,gaze_point_array[30];
Mat xyz;

std::string intrinsic_filename = "/home/macaca/macaca/data/reconstruction/intrinsics.yml";
std::string extrinsic_filename = "/home/macaca/macaca/data/reconstruction/extrinsics.yml";

int SADWindowSize=7, numberOfDisparities=256;//15,32//better:5,256//
float scale=1;

//Ptr<StereoBM> bm = StereoBM::create(16, 9);//originally from reconstructure.h
Ptr<StereoBM> bm = StereoBM::create(16, 9);//originally from reconstructure.h
Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);

Mat img1, img2;
Mat map11, map12, map21, map22;
Size img_size(1280,720);

Rect roi1, roi2;
Mat Q;
Mat disp(720,1280,CV_16SC1), disp8;

cv::Mat img1_raw,img2_raw;

struct detected_object
{
	string classname;
	double probability;
	Rect2i bounding_box;
};
vector<detected_object> detected_object_array;

Rect tracking_box;
Rect2d tracked_box(0,0,0,0);

//Ptr<Tracker> tracker=Tracker::create("MIL");
Ptr<TrackerKCF> tracker = TrackerKCF::create();//KCF:loss and found, MedianFlow:become bigger//loss:MIL,Boosting  slow:TLD,error:GOTURN

void ImagePoint_callback(const eyetracking_msgs::ImagePoint::ConstPtr& msg) 
{ 
    ROS_INFO_STREAM("gaze_point: " <<msg->x<<","<<msg->y); 
	gaze_point_array[29].x=msg->x;
	gaze_point_array[29].y=msg->y;
	for(int i=0;i<29;i++)
	{
		gaze_point_array[i]=gaze_point_array[i+1];
	}
} 

void BoundingBox_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg) 
{
	int num=msg->bounding_boxes.size();
	cout<<"num of bounding_box: "<<num<<endl;
	detected_object_array.clear();
	static bool bounding_box_record=true;//record the first detected bounding_box for tracking
	for(int i=0;i<num;i++)
	{
		if(msg->bounding_boxes[i].Class=="sports ball")
		{
			detected_object temp;
			temp.classname=msg->bounding_boxes[i].Class;
			temp.probability=msg->bounding_boxes[i].probability;
			temp.bounding_box=Rect2i(msg->bounding_boxes[i].xmin,msg->bounding_boxes[i].ymin,msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin,msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin);//tl_x,tl_y,width,height
			detected_object_array.push_back(temp);
			cout<<"sports ball detected"<<endl;

			if(bounding_box_record)
			{
				tracking_box=temp.bounding_box;
				bounding_box_record=false;
			}
		}
	}
}

void ImageCallback_left(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        img1_raw=cv_bridge::toCvCopy(msg,"bgr8")->image;
        //cv::imshow("left_scene",img1_raw);
        //cv::waitKey(1);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("couldn't convert fron '%s' to 'bgr8'.",msg->encoding.c_str());
    }
}
void ImageCallback_right(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        img2_raw=cv_bridge::toCvCopy(msg,"bgr8")->image;
        //cv::imshow("right_scene",img2_raw);
        //cv::waitKey(1);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("couldn't convert fron '%s' to 'bgr8'.",msg->encoding.c_str());
    }
}

static void onMouse(int event, int x, int y, int, void*)
{
	if (selectObject)
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);
	}

	switch (event)
	{
	case EVENT_LBUTTONDOWN:   //鼠标左按钮按下的事件
		origin = Point(x, y);
		selection = Rect(x, y, 0, 0);
		selectObject = true;

		cout << origin << "in world coordinate is: [" << xyz.at<Vec3f>(origin)[0]<<","<<-xyz.at<Vec3f>(origin)[1]<<","<<xyz.at<Vec3f>(origin)[2]<<"]"<< endl;
		cout << origin << "corresponding disparity: " << 16*disp.at<short>(origin)<< endl;
		break;
	case EVENT_LBUTTONUP:    //鼠标左按钮释放的事件
		selectObject = false;
		if (selection.width > 0 && selection.height > 0)
			break;
	}
}

void stereo_calibrate_initialize()
{
	FileStorage fs(intrinsic_filename, FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", intrinsic_filename.c_str());
		return;
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
		return;
	}

	Mat R, T, R1, P1, R2, P2;
	fs["R"] >> R;
	fs["T"] >> T;

	stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);
	cout<<"roi1.size: "<<roi1.size()<<endl;
	cout<<"roi2.size: "<<roi2.size()<<endl;
	cout<<"matrix M1: "<<M1<<endl;
	cout<<"matrix D1: "<<D1<<endl;
	cout<<"matrix R1: "<<R1<<endl;
	cout<<"matrix R2: "<<R2<<endl;
	cout<<"matrix Q: "<<Q<<endl;

	initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
	initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);
}


#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <eyetracking_msgs/ImagePoint.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <image_transport/image_transport.h>

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

int SADWindowSize=5, numberOfDisparities=256;//15,32//better:5,256//
float scale=1;

Ptr<StereoBM> bm = StereoBM::create(16, 9);//originally from reconstructure.h
Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);

Mat img1, img2;
Mat map11, map12, map21, map22;
Size img_size(1280,720);

Rect roi1, roi2;
Mat Q;
Mat disp(720,1280,CV_16SC1), disp8;

cv::Mat img1_raw,img2_raw;

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

geometry_msgs::Point FindClosestObject(vector<pair<Point2d,Point3d> > center_position_array,Point2d gaze_point)
{
	int order=0;
	double min_distance=200000000;
	for(int i=0;i<center_position_array.size();i++)
	{
		double x=center_position_array[i].first.x,y=center_position_array[i].first.y;
		double distance=sqrt((x-gaze_point.x)*(x-gaze_point.x)+(y-gaze_point.y)*(y-gaze_point.y));
		if(distance<min_distance)
		{
			min_distance=distance;
			order=i;
		}
	}
	geometry_msgs::Point point;
	point.x=center_position_array[order].second.x;
	point.y=center_position_array[order].second.y;
	point.z=center_position_array[order].second.z;
	return point;
}
geometry_msgs::PointStamped Find_Target_Object(vector<pair<int,Point3d> > label_position_array,Point2d (&gaze_point_array)[30],Mat label,int nccomps,int &num_target_object)
{
	geometry_msgs::PointStamped pos_target;
	pos_target.point.x=pos_target.point.y=pos_target.point.z=0;
	vector<int> count(nccomps,0);
	for(int i=0;i<30;i++)
	{
		if(gaze_point_array[i].x>1&&gaze_point_array[i].y>1)//remove the effect of (0,0)
		{
			int index=label.at<int>(gaze_point_array[i].y,gaze_point_array[i].x);
			count[index]++;
		}
	}
	/*for(int i=1;i<count.size();i++)//0 is the background
	{
		if(count[i]>15)//condition
		{
			for(int j=0;j<label_position_array.size();j++)
			{
				if(label_position_array[j].first==i)
				{
					num_target_object=i;
					pos_target.x=label_position_array[j].second.x;
					pos_target.y=label_position_array[j].second.y;
					pos_target.z=label_position_array[j].second.z;
				}
			}
		}
	}*/
	num_target_object=1;
	pos_target.header.frame_id="lscene_link";
	pos_target.header.stamp=ros::Time();
	pos_target.point.x=label_position_array[0].second.x/1000;
	pos_target.point.y=label_position_array[0].second.y/1000;
	pos_target.point.z=label_position_array[0].second.z/1000;
	return pos_target;
}


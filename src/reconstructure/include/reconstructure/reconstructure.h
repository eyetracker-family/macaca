#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <eyetracking_msgs/ImagePoint.h>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"

//#include <pcl/visualization/cloud_viewer.h>   
//#include <pcl/io/io.h>  
//#include <pcl/io/pcd_io.h>  

#include <iostream>
#include <cmath>
#include <stack>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/statistical_outlier_removal.h>

#include <stdio.h>

using namespace cv;
using namespace std;

static vector<double> observed_object(3,0.0);//object position to send
int num_target_object=0;

bool selectObject;
Rect selection;
Point2d origin,gaze_point_array[30];
Mat xyz;

std::string intrinsic_filename = "/home/macaca/macaca/src/reconstructure/src/intrinsics.yml";
std::string extrinsic_filename = "/home/macaca/macaca/src/reconstructure/src/extrinsics.yml";

int SADWindowSize=5, numberOfDisparities=256;//15,32//better:5,256//
float scale=1;

Ptr<StereoBM> bm = StereoBM::create(16, 9);//originally from reconstructure.h
Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);

Mat img1, img2;

Rect roi1, roi2;
Mat Q;
Mat disp(720,1280,CV_16SC1), disp8;

Mat binary_left,binary_right;
int thresh =150;
int min_area = 700;
int max_area = 70000;

cv_bridge::CvImagePtr cv_ptr_;
cv::Mat img1_raw,img2_raw;
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;

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

void SeedFillNew(const cv::Mat& _binImg, cv::Mat& _lableImg, std::vector<int>& labelAreaMap )  
{  
  // connected component analysis(4-component)  
  // use seed filling algorithm  
  // 1. begin with a forgeground pixel and push its forground neighbors into a stack;  
  // 2. pop the pop pixel on the stack and label it with the same label until the stack is empty  
  //   
  //  forground pixel: _binImg(x,y)=1  
  //  background pixel: _binImg(x,y) = 0  

  if(_binImg.empty() ||  
     _binImg.type()!=CV_8UC1)  
  {  
    return;  
  }   

  _lableImg.release();  
  _binImg.convertTo(_lableImg,CV_32SC1);  

  int label = 0; //start by 1  
  labelAreaMap.clear();
  labelAreaMap.push_back(0);

  int rows = _binImg.rows;  
  int cols = _binImg.cols;  

  Mat mask(rows, cols, CV_8UC1);  //mask is used to determine if the pixel has been visited,1 for visited and 0 for unvisited.
  mask.setTo(0);  
  int *lableptr;  
  for(int i=0; i < rows; i++)  
  {  
    int* data = _lableImg.ptr<int>(i);  
    uchar *masKptr = mask.ptr<uchar>(i);
    for(int j = 0; j < cols; j++)  
    {  
      if(data[j] == 255&&mask.at<uchar>(i,j)!=1)  
      {  
        mask.at<uchar>(i,j)=1;  
        std::stack<std::pair<int,int>> neighborPixels;  
        neighborPixels.push(std::pair<int,int>(i,j)); // pixel position: <i,j>  
        ++label; //begin with a new label  
        int area = 0;
        while(!neighborPixels.empty())  
        {  
          //get the top pixel on the stack and label it with the same label  
          std::pair<int,int> curPixel =neighborPixels.top();  
          int curY = curPixel.first;  
          int curX = curPixel.second;  
          _lableImg.at<int>(curY, curX) = label;  

          //pop the top pixel  
          neighborPixels.pop();  

          //push the 4-neighbors(foreground pixels)  

          if(curX-1 >= 0)  
          {  
            if(_lableImg.at<int>(curY,curX-1) == 255&&mask.at<uchar>(curY,curX-1)!=1) //leftpixel  
            {  
              neighborPixels.push(std::pair<int,int>(curY,curX-1));  
              mask.at<uchar>(curY,curX-1)=1;  
              area++;
            }  
          }  
          if(curX+1 <=cols-1)  
          {  
            if(_lableImg.at<int>(curY,curX+1) == 255&&mask.at<uchar>(curY,curX+1)!=1)  
              // right pixel  
            {  
              neighborPixels.push(std::pair<int,int>(curY,curX+1));  
              mask.at<uchar>(curY,curX+1)=1;  
              area++;
            }  
          }  
          if(curY-1 >= 0)  
          {  
            if(_lableImg.at<int>(curY-1,curX) == 255&&mask.at<uchar>(curY-1,curX)!=1)  
              // up pixel  
            {  
              neighborPixels.push(std::pair<int,int>(curY-1, curX));  
              mask.at<uchar>(curY-1,curX)=1;  
              area++;
            }    
          }  
          if(curY+1 <= rows-1)  
          {  
            if(_lableImg.at<int>(curY+1,curX) == 255&&mask.at<uchar>(curY+1,curX)!=1)  
              //down pixel  
            {  
              neighborPixels.push(std::pair<int,int>(curY+1,curX));  
              mask.at<uchar>(curY+1,curX)=1;  
              area++;
            }  
          }  
        }  
        labelAreaMap.push_back(area);
      }  
    }  
  }  
}  

cv::Point GetColorBlockCenter(const cv::Mat& rgb, cv::Mat& binary, int thresh, int min_area, int max_area) {
  cv::Point ret(0, 0);
/*
  cv::Mat hsv3;
  std::vector<cv::Mat> hsv;
  cv::cvtColor(rgb, hsv3, cv::COLOR_RGB2HSV);
  cv::split(hsv3, hsv);
  vector<Vec3f> circles;
  cv::Mat hssub = (hsv[1] -hsv[0])*0.5;
  cv::threshold(hssub, binary, thresh, 255, cv::THRESH_BINARY);*/

cv::Mat grey;
cv::cvtColor(rgb,grey,cv::COLOR_RGB2GRAY);
cv::threshold(grey,binary,thresh,255,cv::THRESH_BINARY);
  // TODO 腐蚀膨胀

  cv::Mat label;
  std::vector<int> labelAreaMap;
  SeedFillNew(binary, label,labelAreaMap);

  std::vector<std::pair<int, int>> labelAreas;
  for (int i=1; i < labelAreaMap.size();++i) {
    int area = labelAreaMap[i];
    if (area > min_area && area < max_area) {
      labelAreas.push_back(std::pair<int,int>(labelAreaMap[i], i));
    }
  }

  if (!labelAreas.empty()) {
    std::sort(labelAreas.begin(), labelAreas.end(), [](const std::pair<int, int>&a,const std::pair<int, int>& b) {
      return a.first > b.first;
    });
    long long x = 0, y = 0, count = 0;
    for (int i = 0; i < label.rows; ++i) {
      for (int j =0; j < label.cols;++j) {
        int value = label.at<int>(i,j);
        if (value != labelAreas[0].second)  binary.at<uchar>(i,j) = 0;//if not the biggest area,set the pixel to 0(black),reserve only the biggest area in white.
        else {
          x += j;
          y += i;
          count++;
        }
      }
    }
    if (count != 0) {
      ret = cv::Point(x/count, y/count);
    }
  }
  else {
    binary.setTo(0);
  }


  return ret;
}

vector<std::pair<cv::Point,long long>> GetMultiColorBlockCenter(const cv::Mat& rgb, cv::Mat& binary, int thresh, int min_area, int max_area) {
  vector<std::pair<cv::Point,long long>> center_area;
/*
  cv::Mat hsv3;
  std::vector<cv::Mat> hsv;
  cv::cvtColor(rgb, hsv3, cv::COLOR_RGB2HSV);
  cv::split(hsv3, hsv);
  vector<Vec3f> circles;
  cv::Mat hssub = (hsv[1] -hsv[0])*0.5;
  cv::threshold(hssub, binary, thresh, 255, cv::THRESH_BINARY);*/

	cv::Mat grey;
	cv::cvtColor(rgb,grey,cv::COLOR_RGB2GRAY);
	cv::threshold(grey,binary,thresh,255,cv::THRESH_BINARY);
  // TODO 腐蚀膨胀

  cv::Mat label;
  std::vector<int> labelAreaMap;
  SeedFillNew(binary, label,labelAreaMap);

  std::vector<std::pair<int, int>> labelAreas;
  for (int i=1; i < labelAreaMap.size();++i) {
    int area = labelAreaMap[i];
    if (area > min_area && area < max_area) {
      labelAreas.push_back(std::pair<int,int>(labelAreaMap[i], i));
    }
  }

  if (!labelAreas.empty()) {
    std::sort(labelAreas.begin(), labelAreas.end(), [](const std::pair<int, int>&a,const std::pair<int, int>& b) {
      return a.first > b.first;
    });

	vector<int> labels;//for searching
	for(int k=0;k<labelAreas.size();k++) labels.push_back(labelAreas[k].second);

    vector<long long> x(labelAreaMap.size(),0),y(labelAreaMap.size(),0),count(labelAreaMap.size(),0);
	
    for (int i = 0; i < label.rows; ++i) 
	{
      for (int j =0; j < label.cols;++j) 
	  {
        int value = label.at<int>(i,j);
        //if (value != labelAreas[0].second)  binary.at<uchar>(i,j) = 0;
		if(find(labels.begin(),labels.end(),value)==labels.end()) 
			binary.at<uchar>(i,j) = 0;
		else
	    {
          x[value] += j;
          y[value] += i;
          count[value]++;
        }
      }
    }
	for(int m=0;m<count.size();m++)
	{
		if (count[m] != 0) 
		{
		  center_area.push_back(std::pair<cv::Point,long long>(cv::Point(x[m]/count[m], y[m]/count[m]),count[m]));
		}
	}
  }
  else {
    binary.setTo(0);
  }
  return center_area;
}

geometry_msgs::Point FindClosestObject(vector<pair<Point2d,Point3d>> center_position_array,Point2d gaze_point)
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
geometry_msgs::PointStamped Find_Target_Object(vector<pair<int,Point3d>> label_position_array,Point2d (&gaze_point_array)[30],Mat label,int nccomps,int &num_target_object)
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


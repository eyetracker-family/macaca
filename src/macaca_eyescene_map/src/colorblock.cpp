#include <ros/ros.h>
#include <termios.h>
#include <ar_track_alvar/MarkerPoseService.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>

#include <dynamic_reconfigure/server.h>
#include <tf/transform_datatypes.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <eyetracking_msgs/RotatedRect.h>
#include <macaca_eyescene_map/ColorBlockConfig.h>

#include <opencv2/opencv.hpp>
#include <iostream>
#include <stack>
#include <algorithm>

#include "opencv2/calib3d/calib3d.hpp"

using namespace std;
using namespace cv;

std::string intrinsic_filename = "/home/macaca/macaca/src/reconstructure/src/intrinsics.yml";
std::string extrinsic_filename = "/home/macaca/macaca/src/reconstructure/src/extrinsics.yml";
Mat map11, map12;

bool leftsceneDone = false;
bool lefteyeDone = false;
bool righteyeDone = false;

eyetracking_msgs::RotatedRect left_eye_ellipse;
eyetracking_msgs::RotatedRect right_eye_ellipse;

// useful data
cv::Point2d left_eye_point2d;
cv::Point2d right_eye_point2d;
cv::Point2d left_scene_point2d;
std::vector<cv::Point2d> left_eye_point2d_array;
std::vector<cv::Point2d> right_eye_point2d_array;
std::vector<cv::Point2d> left_scene_point2d_array;

ros::Publisher image_pub;
cv::Mat binary;

int thresh = 70;
int min_area = 700;
int max_area = 7000;


void configCallback(macaca_eyescene_map::ColorBlockConfig &config, uint32_t) {
  ROS_INFO("Reconfigure request : %i %i %i",
           config.thresh,
           config.min_area,
           config.max_area
          );
  thresh = config.thresh;
  min_area = config.min_area;
  max_area = config.max_area;
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

  Mat mask(rows, cols, CV_8UC1);  
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

  cv::Mat hsv3;
  std::vector<cv::Mat> hsv;
  cv::cvtColor(rgb, hsv3, cv::COLOR_RGB2HSV);
  cv::split(hsv3, hsv);
  vector<Vec3f> circles;
  cv::Mat hssub = (hsv[1] -hsv[0])*0.5;
  cv::threshold(hssub, binary, thresh, 255, cv::THRESH_BINARY);
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
        if (value != labelAreas[0].second)  binary.at<uchar>(i,j) = 0;
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

int WriteCalibrationData(std::string filename) {
  int count = left_eye_point2d_array.size();
  if (count == 0) return 0;
  else {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (fs.isOpened()) {
      fs << "count" << count;
      for (int i = 0; i < count; ++i) {
        std::stringstream ss2;
        ss2 << "left_eye_point2d_" << i;
        fs << ss2.str() << left_eye_point2d_array[i];
        std::stringstream ss3;
        ss3 << "right_eye_point2d_" << i;
        fs << ss3.str() << right_eye_point2d_array[i];
        std::stringstream ss4;
        ss4 << "left_scene_point2d_" << i;
        fs << ss4.str() << left_scene_point2d_array[i];
      }
      fs.release();
    } else {
      std::cerr << "failed to open output file " << filename << "\n";
    }
  } 
}


// function getch is from http://answers.ros.org/question/63491/keyboard-key-pressed/
int getch()
{
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if(rv == -1)
        ROS_ERROR("select");
    else if(rv != 0)
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
}
void leftpupilCallback(const eyetracking_msgs::RotatedRectConstPtr& msg) {
  left_eye_point2d = cv::Point2d(msg->x, msg->y);
  if (msg->x == 0.0 && msg->y == 0.0) {
    lefteyeDone = false;
  }
  else {
    lefteyeDone = true;
  }
}

void rightpupilCallback(const eyetracking_msgs::RotatedRectConstPtr& msg) {
  right_eye_point2d = cv::Point2d(msg->x, msg->y);
  if (msg->x == 0.0 && msg->y == 0.0) {
    righteyeDone = false;
  }
  else {
    righteyeDone = true;
  }

}

void remap_init(Mat &map11, Mat &map12)
{
	FileStorage fs(intrinsic_filename, FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", intrinsic_filename.c_str());
	}

	Mat M1, D1, M2, D2;
	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;

	M1 *= 1;
	M2 *= 1;

	fs.open(extrinsic_filename, FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", extrinsic_filename.c_str());
	}

	Mat R, T, R1, P1, R2, P2,Q;
	fs["R"] >> R;
	fs["T"] >> T;

	Size img_size(1280,720);
	Rect roi1, roi2;

	stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

	initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
	cv::Mat cv_image = cv_bridge::toCvCopy(msg, "rgb8")->image;

	remap_init(map11,map12);
	remap(cv_image, cv_image, map11, map12, INTER_LINEAR);

  cv::Point center = GetColorBlockCenter(cv_image, binary, thresh, min_area, max_area);
  if (center.x != 0 || center.y !=0) {
    left_scene_point2d.x = center.x;
    left_scene_point2d.y = center.y;
    leftsceneDone = true;
  }

  /*for(int i=256;i<1280;i+=256)
	for(int j=144;j<720;j+=144)
		circle(cv_image,Point(i,j),5,Scalar(0,255,0),3,8,0);*/
  int border=150,rows=3,cols=4;
  for(int i=border;i<=1280-border;i+=(1280-2*border)/(cols-1))
	for(int j=border;j<=720-border;j+=(720-2*border)/(rows-1))
		circle(cv_image,Point(i,j),5,Scalar(0,255,0),3,8,0);

  cv::circle(cv_image,center,5,Scalar(255,0,0),3,8,0);
  imshow("gaussian process regression calibrate",cv_image);
  cv::waitKey(10);

  sensor_msgs::Image binmsg;
  cv_bridge::CvImage cv_bri_img(msg->header, sensor_msgs::image_encodings::MONO8, binary);
  image_pub.publish(cv_bri_img.toImageMsg());
//cout<<"binary size: "<<binary.size()<<endl;
}



int main(int argc, char *argv[])
{
  ros::init(argc, argv, "markerPoseClient");
  ros::NodeHandle nh("~");
  std::string image_topic = "/scene/left/image_color";
  std::string output_filename = "/home/volcanoh/macaca/data/eyescene_map/ColorBlock.yml";

  std::string left_pupil_topic = "/eye/left/pupil_ellipse";
  std::string right_pupil_topic = "/eye/right/pupil_ellipse";

  //nh.getParam("image_topic", image_topic);
  nh.getParam("output_filename", output_filename);
  nh.getParam("thresh", thresh);
  nh.getParam("min_area", min_area);
  nh.getParam("max_area", max_area);

  ROS_INFO("subscribing image topic %s", image_topic.c_str());
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub = it.subscribe(image_topic, 10, imageCallback);
  ros::Subscriber left_pupil_sub = nh.subscribe(left_pupil_topic, 1, leftpupilCallback);
  ros::Subscriber right_pupil_sub = nh.subscribe(right_pupil_topic, 1, rightpupilCallback);
  image_pub = nh.advertise<sensor_msgs::Image>("/scene/left/image_rect_binary", 1);

  dynamic_reconfigure::Server<macaca_eyescene_map::ColorBlockConfig> dyn_server;
  dynamic_reconfigure::Server<macaca_eyescene_map::ColorBlockConfig>::CallbackType dyn_callback;
  dyn_callback = boost::bind(&configCallback, _1, _2);
  dyn_server.setCallback(dyn_callback);


  ros::Rate r(30);
  ros::Duration(1.0).sleep(); // wait for the first callback
  while (ros::ok()) {
    ros::spinOnce();

    int key = getch();
    //int key = waitKey(33);
    if (lefteyeDone && leftsceneDone && righteyeDone) {
      if ((key == 's') || (key == 'S')){
        std::cout << "========================================" << std::endl;
        lefteyeDone = false;
        righteyeDone = false;
        leftsceneDone = false;

        left_eye_point2d_array.push_back(left_eye_point2d);
        right_eye_point2d_array.push_back(right_eye_point2d);
        left_scene_point2d_array.push_back(left_scene_point2d);
        std::cout << "lef eye Point2d " << left_eye_point2d << std::endl;
        std::cout << "rig eye Point2d " << right_eye_point2d << std::endl;
        std::cout << "lef sce Point2d " << left_scene_point2d << std::endl;

        WriteCalibrationData(output_filename);
        std::cout << "Add Data, size ==  #:" << left_scene_point2d_array.size() << "\n";
      }
      else if ((key == 'd') || (key == 'D')) {
        if (left_scene_point2d_array.empty()) {
          std::cout << "Data size == " << left_scene_point2d_array.size() << "\n";
        }
        else {
          left_eye_point2d_array.pop_back();
          right_eye_point2d_array.pop_back();
          left_scene_point2d_array.pop_back();
			WriteCalibrationData(output_filename);
          std::cout << "Delete Data, size == #:" << left_scene_point2d_array.size() << "\n";
        }
      }
      else if ((key == 'q') || (key == 'Q')) {
        break;
      }

    }
    else {
      //ROS_INFO("Call Failed");
      if ((key == 's') || (key == 'S')) {
        std::cout << "========================================" << std::endl;
        if (!lefteyeDone) {
          ROS_INFO("lefteye is not Done");
        }
        if (!righteyeDone) {
          ROS_INFO("righteye is not Done");
        }
        if (!leftsceneDone) {
          ROS_INFO("leftscene is not Done");
        }
      }
      else if ((key == 'd') || (key == 'D')) {
        if (left_scene_point2d_array.empty()) {
          std::cout << "Data size == " << left_scene_point2d_array.size() << "\n";
        }
        else {
          std::cout << "Delete Data, size == #:" << left_scene_point2d_array.size() << "\n";
          left_eye_point2d_array.pop_back();
          right_eye_point2d_array.pop_back();
          left_scene_point2d_array.pop_back();
        }
      }
      else if ((key == 'q') || (key == 'Q')) {
        break;
      }
    }
  }

  return 0;
}

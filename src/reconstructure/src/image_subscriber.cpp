#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
cv::Mat image;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
	image=cv_bridge::toCvCopy(msg, "bgr8")->image;
     //cv::imshow("subscriber", cv_bridge::toCvShare(msg, "bgr8")->image);
	imshow("subscriber",image);
	cout<<"aaa"<<endl;
     cv::waitKey(1);
   }
   catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
}
 
int main(int argc, char **argv)
{
   ros::init(argc, argv, "image_listener");
   ros::NodeHandle nh;
   //cv::namedWindow("view");
   cv::startWindowThread();
   image_transport::ImageTransport it(nh);
   //image_transport::Subscriber sub = it.subscribe("/image_publisher/camera/image", 1, imageCallback);
   image_transport::Subscriber sub = it.subscribe("scene/left/image_color", 1, imageCallback);
cout<<"bbb"<<endl;
	while(nh.ok())
    {
		//cout<<"bbb"<<endl;
		ros::spinOnce();
	}
   //ros::spin();
   //cv::destroyWindow("view");
 }

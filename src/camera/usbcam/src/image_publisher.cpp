#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer
#include <boost/shared_ptr.hpp>
#include <string>

using namespace std;
using namespace cv;

int main(int argc, char** argv)
{
   // Check if video source has been passed as a parameter
   //if(argv[1] == NULL) return 1;
 
   ros::init(argc, argv, "image_publisher");
   ros::NodeHandle nh("~");//

cout<<"12345hhhh"<<endl;

   image_transport::ImageTransport it(nh);

  const std::string image_topic_name = "camera/image";
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  //boost::shared_ptr<image_transport::CameraPublisher> pub(new image_transport::CameraPublisher(it.advertiseCamera(image_topic_name, 5)));

  cv::VideoCapture cap(1);
  cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
 
   if(!cap.isOpened()) return 1;
	else cout<<"camera is opened"<<endl;
   cv::Mat frame;
   sensor_msgs::ImagePtr msg;
 
   //ros::Rate loop_rate(5);
cout<<"1hhhh"<<endl;
   while (nh.ok()) {
     cap >> frame;
cout<<"2hhhh"<<endl;
     // Check if grabbed frame is actually full with some content
     if(!frame.empty()) {
       msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
       pub.publish(msg);
cv::imshow("view", frame);
cout<<"3hhhh"<<endl;
       cv::waitKey(1);
     }
 
     ros::spinOnce();
     //loop_rate.sleep();
   }
 }

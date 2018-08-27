#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/shared_ptr.hpp>
#include <string>

using namespace cv;
using namespace std;

int main(int argc, char** argv) {

  ros::init(argc, argv, "usbcam_node");
  cout << "init ros node" << endl;
  ros::NodeHandle nh("~");
  image_transport::ImageTransport it(nh);

  int camera_id = 0;
  std::string url = "";
  std::string camera_name = "camera";
  nh.getParam("camera_id", camera_id);
  nh.getParam("camera_info_url", url);
  nh.getParam("camera_name", camera_name);

  const std::string image_topic_name = "image_raw";

  boost::shared_ptr<image_transport::CameraPublisher> pub(new image_transport::CameraPublisher(it.advertiseCamera(image_topic_name, 5)));

  cv::VideoCapture frame(camera_id);
  frame.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
  frame.set(CAP_PROP_FRAME_WIDTH,1280);
  frame.set(CAP_PROP_FRAME_HEIGHT,720);

  //frame.set(CAP_PROP_FRAME_WIDTH,640);
  //frame.set(CAP_PROP_FRAME_HEIGHT,480);
  if(frame.isOpened()) {
    cout << "Camera " << camera_id << " is Opened.\nTopic: " << image_topic_name << endl;
  }
  else {
    cerr << "Camera " << camera_id << " is not found" << endl;
    return -1;
  }

  cv::Mat image;
  sensor_msgs::ImagePtr msg;
  camera_info_manager::CameraInfoManager cinfo(nh);
  if (cinfo.validateURL(url))
    cinfo.loadCameraInfo(url);
  else {
    cout << "Invalid camera calibration URL: " << url << endl;
    return -1;
  }

  sensor_msgs::CameraInfoPtr info(new sensor_msgs::CameraInfo(cinfo.getCameraInfo()));
  std_msgs::Header header;
  header.frame_id = ros::names::resolve(ros::this_node::getNamespace() ,camera_name) + "/camera_link";
  info->header.frame_id = header.frame_id;
  while (nh.ok()) {
    frame >> image;
	//cout<<"image size: "<<image.size()<<endl;
    if (image.empty()) continue;
    ros::Time time = ros::Time::now();
    header.stamp = time;
    info->header.stamp = time;
    msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    pub->publish(msg, info);
  }
  return 0;
} 


#include <ros/ros.h>
#include <ar_track_alvar/MarkerPoseService.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>

#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/MultiMarkerBundle.h"
#include "ar_track_alvar/MultiMarkerInitializer.h"
#include "ar_track_alvar/Shared.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_datatypes.h>
#include <image_transport/image_transport.h>

sensor_msgs::Image image;
sensor_msgs::CameraInfo camera_info;
bool imageDone = false;
bool camerainfoDone = false;

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  imageDone = true;
  image = *msg;
}

void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
  camerainfoDone = true;
  camera_info = *msg;
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "marker_pose_client");
  ros::NodeHandle nh;
  std::string image_topic = "/scene/right/image_rect_color";
  std::string camera_info_topic = "/scene/right/camera_info";
  std::string bundle_xml_file = "/home/volcanoh/macaca/src/ar_track_alvar/ar_track_alvar/bundles/MarkerData_0-8_5.4.xml";
  double marker_size = 5.4;
  double max_new_marker_error = 0.08;
  double max_track_error = 0.2;

  nh.getParam("image_topic", image_topic);
  nh.getParam("camera_info_topic", camera_info_topic);
  nh.getParam("bundle_xml_file", bundle_xml_file);
  nh.getParam("marker_size", marker_size);
  nh.getParam("max_new_marker_error", max_new_marker_error);
  nh.getParam("max_track_error", max_track_error);

  ROS_INFO("subscribing image topic %s", image_topic.c_str());
  ROS_INFO("subscribing camera info %s", camera_info_topic.c_str());
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub = it.subscribe(image_topic, 10, imageCallback);
  ros::Subscriber camera_info_sub = nh.subscribe(camera_info_topic, 10, camerainfoCallback);

  do {
    ros::spinOnce();
    if (!imageDone || !camerainfoDone){
      continue;
    } 
    ros::ServiceClient client = nh.serviceClient<ar_track_alvar::MarkerPoseService>("marker_pose");
    ar_track_alvar::MarkerPoseService srv;
    srv.request.image = image;
    srv.request.camera_info = camera_info;
    srv.request.bundles_xml_file.data = bundle_xml_file;
    srv.request.marker_size = marker_size;
    srv.request.max_new_marker_error = max_new_marker_error;
    srv.request.max_track_error = max_track_error;

    if (client.call(srv)) {
      ROS_INFO("Call Success");
      tf::Transform pose;
      tf::poseMsgToTF(srv.response.camera_to_artag, pose);
      tf::Vector3 vec = pose.getOrigin();
      tf::Quaternion rotation = pose.getRotation();
      std::cout << "======================================" << std::endl;
      std::cout << "vec x,y,z\n" << vec.x() << " " << vec.y() << " " << vec.z() << std::endl;
      std::cout << "rotation x y z w\n" <<  rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w() << std::endl;
      std::cout << "corner points image" << std::endl;
      for (size_t i = 0; i < srv.response.points_num; ++i) {
        std::cout << srv.response.x[i] << ",";
        std::cout << srv.response.y[i] << " | ";
      }std::cout << std::endl;
      std::cout << "======================================" << std::endl;
    }
    else {
      ROS_INFO("Call Failed");
    }
  } while (!(imageDone && camerainfoDone) && ros::ok());
  return 0;
}

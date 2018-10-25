#include "lighthouse_track/track_object.h"
#include <thread>
#include <fstream>
using namespace std;

// ros header files
#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


bool ReadPoints(std::vector<cv::Point3d>& pts, std::string file) {
  int nums = 0;
  std::ifstream fin(file, std::fstream::in);
  if (!fin.is_open()) return false;
  fin >> nums;
  double x = 0, y = 0, z = 0;
  for (int i = 0; i < nums; ++i) {
    fin >> x >> y >> z;
    pts.push_back(cv::Point3d(x/1000.0, y/1000.0, z/1000.0));
  }
  return true;
}

int main(int argc, char* argv[]) {

  ros::init(argc, argv, "track_marker_node");
  ros::NodeHandle nh("~");

  std::string usb_device_name ="/dev/ttyUSB0";
  std::string object_points_file = "object_points.txt";
  std::string lighthouse_frame = "pnp_link";
  std::string marker_frame = "marker_link";
  std::string world_frame = "map";


  nh.getParam("usb_device_name", usb_device_name);
  nh.getParam("object_points_file", object_points_file);
  nh.getParam("lighthouse_frame", lighthouse_frame);
  nh.getParam("marker_frame", marker_frame);
  nh.getParam("world_frame", world_frame);

  cout << usb_device_name << " " << object_points_file << endl;
  UsbSerialLinux usb_serial(usb_device_name);
  if (!usb_serial.IsOpened()) {
    ROS_ERROR("Can not open device %s. ", usb_device_name.c_str());
    return -1;
  } 

  std::vector<cv::Point3d> vertices;
  if (!ReadPoints(vertices, object_points_file)) {
    ROS_ERROR("Can not open file %s. ", object_points_file.c_str());
    return -1;
  }
  // ObjectManager test
  TrackObject track_object(usb_serial, vertices);
  cout << "Starting lighthouse tracking" << endl;
  track_object.Start();

  tf::TransformBroadcaster *tf_broadcaster = new tf::TransformBroadcaster();
  tf::TransformListener* tf_listener = new tf::TransformListener(nh);
  while(nh.ok()) {

    ros::Time time = ros::Time::now();
    tf::StampedTransform lighthouse_to_world;
    try {
      tf_listener->waitForTransform(world_frame, lighthouse_frame, time, ros::Duration(1.0));//ros::Time(0) is better(perfect). ros::Time(0) means to get the latest available tf while ros::Time::now() means to get the tf at the current time,so it needs to wait for the tf broadcaster.
	//so use waitForTransform+ros::Time::now() or simply ros::Time(0)
      tf_listener->lookupTransform(world_frame, lighthouse_frame, time, lighthouse_to_world);
    }
    catch(tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
    }

    cv::Matx31d rvec, tvec;
    track_object.GetTrackRvec(rvec);
    track_object.GetTrackTvec(tvec);

    tf::Quaternion rotation(rvec(0), rvec(1), rvec(2));
    tf::Vector3 origin(tvec(0), tvec(1), tvec(2));
    if (!isnan(rotation.x()) && !isnan(rotation.y())&& !isnan(rotation.z()) && !isnan(rotation.w()) && std::abs(origin.x()) < 5 && std::abs(origin.y()) < 5 && abs(origin.z()) < 5 ) {
      tf::Transform t(rotation, origin);
      tf::StampedTransform EyetrackerToLighthouse(t, time, lighthouse_frame, marker_frame);
      tf_broadcaster->sendTransform(EyetrackerToLighthouse);
    }
    usleep(1000);
  }
  //track_object.Join();
  usb_serial.Close();
  return 0;
}

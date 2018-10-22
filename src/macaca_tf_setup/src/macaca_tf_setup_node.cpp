#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>


enum EyeTrackerTo {
  LeftSceneCamera = 0,
  RightSceneCamera,
  LeftEyeCamera,
  RightEyeCamera
};

bool ReadTfData(const std::string& file, tf::Transform& t) {
  cv::FileStorage fs;
  try {
    ROS_INFO("Opening %s", file.c_str());
    fs.open(file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
      ROS_ERROR("Cannot open file: %s", file.c_str());
      return false;
    }
    cv::Mat mat4d;
    fs["ArmTipToMarkerTagTransform"] >> mat4d;
    fs.release();

    tf::Vector3 origin;
    tf::Matrix3x3 tf3d;

    origin.setValue (mat4d.at<double>(0, 3),
                     mat4d.at<double>(1, 3),
                     mat4d.at<double>(2, 3)
                    );
    tf3d.setValue(
      mat4d.at<double>(0, 0), mat4d.at<double>(0, 1), mat4d.at<double>(0, 2),
      mat4d.at<double>(1, 0), mat4d.at<double>(1, 1), mat4d.at<double>(1, 2),
      mat4d.at<double>(2, 0), mat4d.at<double>(2, 1), mat4d.at<double>(2, 2)
      );
    tf::Quaternion rotation;
    tf3d.getRotation(rotation);
    t.setOrigin(origin);
    t.setRotation(rotation);

  } catch(cv::Exception ex) {
  }
  return true;
}

std::string eyetracker_link = "eyetracker_link";
std::string scene_left_camera_link = "lscene_link";///scene/left/camera_link
std::string scene_right_camera_link = "/scene/right/camera_link";
std::string eye_left_camera_link = "/eye/left/camera_link";
std::string eye_right_camera_link = "/eye/right/camera_link";

std::string scene_left_tf_datafile = "";
std::string scene_right_tf_datafile = "";
std::string eye_left_tf_datafile = "";
std::string eye_right_tf_datafile = "";
std::string pnp2lighthouse_tf_datafile = "";
std::string tracker1_robot_tf_datafile = "";

int frame_rate = 10;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "macaca_tf_setup");
  ros::NodeHandle nh("~");

  nh.getParam("rate", frame_rate);
  nh.getParam("scene_left_tf_datafile", scene_left_tf_datafile);
  nh.getParam("scene_right_tf_datafile", scene_right_tf_datafile);
  nh.getParam("eye_left_tf_datafile", eye_left_tf_datafile);
  nh.getParam("eye_right_tf_datafile", eye_right_tf_datafile);
  nh.getParam("pnp2lighthouse_tf_datafile", pnp2lighthouse_tf_datafile);
  nh.getParam("tracker1_robot_tf_datafile", tracker1_robot_tf_datafile);


  ros::Rate rate(frame_rate);

  tf::Transform scene_left_tf;
  tf::Transform scene_right_tf;
  tf::Transform eye_left_tf;
  tf::Transform eye_right_tf;
  tf::Transform pnp2lighthouse_tf;
  tf::Transform tracker1_robot_tf;

  if (!ReadTfData(scene_left_tf_datafile, scene_left_tf) ) {
    return -1;
  }
  if (!ReadTfData(scene_right_tf_datafile, scene_right_tf)) {
    return -1;
  }
  if (!ReadTfData(eye_left_tf_datafile, eye_left_tf)) {
    return -1;
  }
  if (!ReadTfData(eye_right_tf_datafile, eye_right_tf)) {
    return -1;
  }
  if (!ReadTfData(pnp2lighthouse_tf_datafile, pnp2lighthouse_tf)) {
    return -1;
  }
  if (!ReadTfData(tracker1_robot_tf_datafile, tracker1_robot_tf)) {
    return -1;
  }


  tf::TransformBroadcaster broadcaster;

  while(nh.ok()) {
    ros::Time time = ros::Time::now();

    broadcaster.sendTransform(
      tf::StampedTransform(scene_left_tf, time, eyetracker_link, scene_left_camera_link)
      //tf::StampedTransform(scene_left_tf, time, scene_left_camera_link,eyetracker_link )
      );
    /*broadcaster.sendTransform(
      tf::StampedTransform(scene_right_tf, time, eyetracker_link, scene_right_camera_link)
      );
    broadcaster.sendTransform(
      tf::StampedTransform(eye_left_tf, time, eyetracker_link, eye_left_camera_link)
      );
    broadcaster.sendTransform(
      tf::StampedTransform(eye_right_tf, time, eyetracker_link, eye_right_camera_link)
      );*/
    broadcaster.sendTransform(
      tf::StampedTransform(pnp2lighthouse_tf, time, "lighthouse_link", "pnp_link")
      //tf::StampedTransform(scene_left_tf, time, scene_left_camera_link,eyetracker_link )
      );
    broadcaster.sendTransform(
      //tf::StampedTransform(tracker1_robot_tf, time, "robot_link", "tracker1_link")
      tf::StampedTransform(tracker1_robot_tf, time, "tracker1_link", "robot_link")
      );

    rate.sleep();
  }
  return 0;
}

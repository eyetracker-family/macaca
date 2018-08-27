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

bool callback(ar_track_alvar::MarkerPoseService::Request & req,
              ar_track_alvar::MarkerPoseService::Response & res) {
  // get image_msg, camera_info msg, xml msg
  ROS_INFO("Being Called");
  sensor_msgs::Image image_msg = req.image;
  sensor_msgs::CameraInfo camera_info = req.camera_info;
  std_msgs::String bundles_xml_file = req.bundles_xml_file;
  double marker_size = req.marker_size;
  double max_new_marker_error = req.max_new_marker_error;
  double max_track_error = req.max_track_error;


  MultiMarker multi_marker;
  multi_marker.Load(req.bundles_xml_file.data.c_str(), FILE_FORMAT_XML);


  // Set up camera
  Camera* cam = new Camera();
  cam->SetCameraInfo(camera_info);

  cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  IplImage ipl_image = cv_image_ptr->image;

  // init marker_detector
  MarkerDetector<MarkerData> marker_detector;
  marker_detector.SetMarkerSize(marker_size);

  if (marker_detector.Detect(&ipl_image, cam, true, false, max_new_marker_error, max_track_error, CVSEQ, true)){
    Pose bundlePose;
    multi_marker.Update(marker_detector.markers, cam, bundlePose);
    double px,py,pz,qx,qy,qz,qw;
    px = bundlePose.translation[0]/100.0;
    py = bundlePose.translation[1]/100.0;
    pz = bundlePose.translation[2]/100.0;
    qx = bundlePose.quaternion[1];
    qy = bundlePose.quaternion[2];
    qz = bundlePose.quaternion[3];
    qw = bundlePose.quaternion[0];

    //Get the marker pose in the camera frame
    tf::Quaternion rotation (qx,qy,qz,qw);
    tf::Vector3 origin (px,py,pz);
    tf::Transform t (rotation, origin);  //transform from cam to marker
    tf::poseTFToMsg(t, res.camera_to_artag);
    int master_id = multi_marker.getMasterId();
    for (size_t i = 0; i < marker_detector.markers->size(); ++i) {
      auto mdm = *marker_detector.markers;
      if (mdm[i].GetId() == master_id) {
        res.points_num = mdm[i].marker_corners_img.size();
        for (size_t j = 0; j < res.points_num; ++j) {
          res.x.push_back(mdm[i].marker_corners_img[j].x);
          res.y.push_back(mdm[i].marker_corners_img[j].y);
        }
      }
    }
    return true;
  }
  else {
    return false;
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "marker_pose_server");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("marker_pose", callback);
  ros::spin();
  return 0;
}

#include <ros/ros.h>
#include <termios.h>
#include <ar_track_alvar/MarkerPoseService.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>

#include <tf/transform_datatypes.h>
#include <image_transport/image_transport.h>
#include <eyetracking_msgs/RotatedRect.h>
#include <opencv2/opencv.hpp>

sensor_msgs::Image image;
sensor_msgs::CameraInfo camera_info;
bool worldPointDone= false;
bool leftsceneDone = false;
bool lefteyeDone = false;
bool righteyeDone = false;

eyetracking_msgs::RotatedRect left_eye_ellipse;
eyetracking_msgs::RotatedRect right_eye_ellipse;

// useful data
cv::Point3d point3d;
cv::Point2d left_eye_point2d;
cv::Point2d right_eye_point2d;
cv::Point2d left_scene_point2d;
std::vector<cv::Point3d> point3d_array;
std::vector<cv::Point2d> left_eye_point2d_array;
std::vector<cv::Point2d> right_eye_point2d_array;
std::vector<cv::Point2d> left_scene_point2d_array;

int WriteCalibrationData(std::string filename) {
  int size = point3d_array.size();
  if (size == 0) return 0;
  else {
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (fs.isOpened()) {
      fs << "count" <<(int) point3d_array.size();
      for (int i = 0; i < size; ++i) {
        std::stringstream ss1;
        ss1 << "point3d_" << i;
        fs << ss1.str() << point3d_array[i];
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
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
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


void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  image = *msg;
}

void camerainfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
  camera_info = *msg;
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "markerPoseClient");
  ros::NodeHandle nh("~");
  std::string image_topic = "/scene/right/image_rect_color";
  std::string camera_info_topic = "/scene/right/camera_info";
  std::string bundle_xml_file = "/home/volcanoh/macaca/src/ar_track_alvar/ar_track_alvar/bundles/MarkerData_0-8_5.4.xml";
  std::string output_filename = "/home/volcanoh/macaca/data/eyescene_map/EyeSceneMapCalibrationData.yml";
  double marker_size = 5.4;
  double max_new_marker_error = 0.08;
  double max_track_error = 0.2;

  std::string left_pupil_topic = "/eye/left/pupil_ellipse";
  std::string right_pupil_topic = "/eye/right/pupil_ellipse";

  nh.getParam("image_topic", image_topic);
  nh.getParam("camera_info_topic", camera_info_topic);
  nh.getParam("bundle_xml_file", bundle_xml_file);
  nh.getParam("marker_size", marker_size);
  nh.getParam("max_new_marker_error", max_new_marker_error);
  nh.getParam("max_track_error", max_track_error);

  nh.getParam("output_filename", output_filename);

  ROS_INFO("subscribing image topic %s", image_topic.c_str());
  ROS_INFO("subscribing camera info %s", camera_info_topic.c_str());
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub = it.subscribe(image_topic, 10, imageCallback);
  ros::Subscriber left_pupil_sub = nh.subscribe(left_pupil_topic, 1, leftpupilCallback);
  ros::Subscriber right_pupil_sub = nh.subscribe(right_pupil_topic, 1, rightpupilCallback);
  ros::Subscriber camera_info_sub = nh.subscribe(camera_info_topic, 10, camerainfoCallback);


  ros::Rate r(30);
  ros::Duration(1.0).sleep(); // wait for the first callback
  while (ros::ok()) {
    ros::spinOnce();
    ros::ServiceClient client = nh.serviceClient<ar_track_alvar::MarkerPoseService>("/marker_pose");
    ar_track_alvar::MarkerPoseService srv;
    srv.request.image = image;
    srv.request.camera_info = camera_info;
    srv.request.bundles_xml_file.data = bundle_xml_file;
    srv.request.marker_size = marker_size;
    srv.request.max_new_marker_error = max_new_marker_error;
    srv.request.max_track_error = max_track_error;

    int key = getch();
    if (lefteyeDone && client.call(srv)) {
      //ROS_INFO("marker_pose Called Success");
      tf::Transform pose;
      tf::poseMsgToTF(srv.response.camera_to_artag, pose);
      tf::Vector3 vec = pose.getOrigin();
      point3d = cv::Point3d(vec.x(), vec.y(), vec.z());
      if (point3d.x == 0.0 && point3d.y == 0.0 && point3d.z == 0.0) {
        worldPointDone = false;
        continue;
      } else {
        worldPointDone = true;
      }
      int points_num = srv.response.points_num;
      if (points_num > 0) {
        left_scene_point2d = cv::Point2d(srv.response.x[points_num-1], srv.response.y[points_num-1]);
        if (left_scene_point2d.x == 0.0 && left_scene_point2d.y == 0.0) {
          leftsceneDone = false;
          continue;
        } else {
          leftsceneDone = true;
        }
      }
      if ((key == 's') || (key == 'S')){

        std::cout << "========================================" << std::endl;
        if (!worldPointDone) {
          ROS_INFO("WorldPoint is not Done");
        }
        if (!lefteyeDone) {
          ROS_INFO("lefteye is not Done");
        }
        if (!righteyeDone) {
          ROS_INFO("righteye is not Done");
        }
        if (!leftsceneDone) {
          ROS_INFO("leftscene is not Done");
        }

        if (worldPointDone && lefteyeDone && righteyeDone && leftsceneDone) {
          worldPointDone = false;
          lefteyeDone = false;
          righteyeDone = false;
          leftsceneDone = false;

          WriteCalibrationData(output_filename);
          std::cout << "Add Data, size ==  #:" << point3d_array.size() << "\n";
          point3d_array.push_back(point3d);
          left_eye_point2d_array.push_back(left_eye_point2d);
          right_eye_point2d_array.push_back(right_eye_point2d);
          left_scene_point2d_array.push_back(left_scene_point2d);
          std::cout << "the wrd Point3d " << point3d << std::endl;
          std::cout << "lef eye Point2d " << left_eye_point2d << std::endl;
          std::cout << "rig eye Point2d " << right_eye_point2d << std::endl;
          std::cout << "lef sce Point2d " << left_scene_point2d << std::endl;
        }
      }
      else if ((key == 'd') || (key == 'D')) {
        if (point3d_array.empty()) {
          std::cout << "Data size == " << point3d_array.size() << "\n";
        }
        else {
          std::cout << "Delete Data, size == #:" << point3d_array.size() << "\n";
          point3d_array.pop_back();
          left_eye_point2d_array.pop_back();
          right_eye_point2d_array.pop_back();
          left_scene_point2d_array.pop_back();
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
        if (!worldPointDone) {
          ROS_INFO("WorldPoint is not Done");
        }
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
        if (point3d_array.empty()) {
          std::cout << "Data size == " << point3d_array.size() << "\n";
        }
        else {
          std::cout << "Delete Data, size == #:" << point3d_array.size() << "\n";
          point3d_array.pop_back();
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

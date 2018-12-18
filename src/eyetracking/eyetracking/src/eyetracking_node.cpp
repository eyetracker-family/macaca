#include "algo.h"
#include "ElSe.h"
#include "ExCuSe.h"
#include "PuRe.h"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pupiltracker/cvx.h>
#include <pupiltracker/PupilTracker.h>

#include <eyetracking/PupilParamsConfig.h>
#include <dynamic_reconfigure/server.h>

#include <eyetracking_msgs/RotatedRect.h>

using namespace cv;
using namespace std;

cv_bridge::CvImagePtr cv_ptr_;
cv::Mat image_;
cv::Point2f pupil_center_;
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;
ros::Publisher pupil_pub_;

eyetracking_msgs::RotatedRect pupil_ellipse_msg;

pupiltracker::TrackerParams params;

const std::string output_topic_="pupil_ellipse";

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try {
    cv_ptr_ = cv_bridge::toCvCopy(msg, "bgr8");
    image_ = cv_ptr_->image;
		if (image_.empty())
		{
			cout << "no image" << endl;
		}

	if(!image_.empty())
	{
		cv::cvtColor(image_, image_, cv::COLOR_BGR2GRAY);
		int size_times = image_.cols / 320; 
		cv::resize(image_, image_, cv::Size(320, image_.rows / size_times));

		try {
		  /*pupiltracker::findPupilEllipse_out out;
		  pupiltracker::tracker_log log;
		  pupiltracker::findPupilEllipse(params, image_, out, log);

		  pupiltracker::cvx::cross(image_, out.pPupil, 5, pupiltracker::cvx::rgb(255, 255, 0));
		  cv::ellipse(image_, out.elPupil, pupiltracker::cvx::rgb(255,0,255));

		  pupil_ellipse_msg.header = msg->header;
		  pupil_ellipse_msg.x = out.elPupil.center.x * size_times;
		  pupil_ellipse_msg.y = out.elPupil.center.y * size_times;
		  pupil_ellipse_msg.angle = out.elPupil.angle;
		  pupil_ellipse_msg.width = out.elPupil.size.width * size_times;
		  pupil_ellipse_msg.height = out.elPupil.size.height * size_times;*/

		  //cv::RotatedRect pupil = ELSE::run(image_);
		  //RotatedRect pupil = ELSE::run(grey);//algo
		  //RotatedRect pupil = ElSe().run(grey);
		  //RotatedRect pupil = ExCuSe().run(grey);
		  RotatedRect pupil = PuRe().run(image_);
		  cv::cvtColor(image_, image_, cv::COLOR_GRAY2BGR);

		  if(pupil.size.height>20)
		  {

			  pupiltracker::cvx::cross(image_, cv::Point2f(pupil.center.x,pupil.center.y), 5, pupiltracker::cvx::rgb(255, 255, 0));
			  cv::ellipse(image_, pupil, pupiltracker::cvx::rgb(255,0,255));

			  pupil_ellipse_msg.header = msg->header;
			  pupil_ellipse_msg.x = pupil.center.x * size_times;
			  pupil_ellipse_msg.y = pupil.center.y * size_times;
			  pupil_ellipse_msg.angle = pupil.angle;
			  pupil_ellipse_msg.width = pupil.size.width * size_times;
			  pupil_ellipse_msg.height = pupil.size.height * size_times;
		  }

		  pupil_pub_.publish(pupil_ellipse_msg);

		  sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(msg->header, "bgr8", image_).toImageMsg();
		  image_pub_.publish(pub_msg);

		}
		catch (cv::Exception e) 
		{
		  ROS_ERROR("Pupil Algorithm Error!");
		}
	}
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
  }
}

void configCallback(eyetracking::PupilParamsConfig &config, uint32_t) {
  ROS_INFO("Reconfigure request : %i %i %i %i %i %i %i %i %s %i %s %i",
           config.Radius_Min,
           config.Radius_Max,
           config.CannyBlur,
           config.CannyThreshold1,
           config.CannyThreshold2,
           config.StarburstPoints,
           config.PercentageInliers,
           config.InlierIterations,
           config.ImageAwareSupport ? "True" : "False",
           config.EarlyTerminationPercentage,
           config.EarlyRejection ? "True" : "False",
           config.Seed
          );
  params.Radius_Min = config.Radius_Min;
  params.Radius_Max= config.Radius_Max;
  params.CannyBlur = config.CannyBlur;
  params.CannyThreshold1 = config.CannyThreshold2;
  params.CannyThreshold2 = config.CannyThreshold2;
  params.StarburstPoints= config.StarburstPoints;
  params.PercentageInliers = config.PercentageInliers;
  params.InlierIterations = config.InlierIterations;
  params.ImageAwareSupport = config.ImageAwareSupport;
  params.EarlyTerminationPercentage = config.EarlyTerminationPercentage;
  params.EarlyRejection = config.EarlyRejection;
  params.Seed = config.Seed;

}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;

  nh.getParam("Radius_Min", params.Radius_Min);
  nh.getParam("Radius_Max", params.Radius_Max);
  nh.getParam("CannyBlur", params.CannyBlur);
  nh.getParam("CannyThreshold1", params.CannyThreshold1);
  nh.getParam("CannyThreshold2", params.CannyThreshold2);

  nh.getParam("StarburstPoints", params.StarburstPoints);
  nh.getParam("InlierIterations", params.InlierIterations);
  nh.getParam("PercentageInliers", params.PercentageInliers);

  nh.getParam("ImageAwareSupport", params.ImageAwareSupport);
  nh.getParam("EarlyTerminationPercentage", params.EarlyTerminationPercentage);
  nh.getParam("EarlyRejection", params.EarlyRejection);
  nh.getParam("Seed", params.Seed);

  dynamic_reconfigure::Server<eyetracking::PupilParamsConfig> dyn_server;
  dynamic_reconfigure::Server<eyetracking::PupilParamsConfig>::CallbackType dyn_callback;
  dyn_callback = boost::bind(&configCallback, _1, _2);
  dyn_server.setCallback(dyn_callback);

  pupil_pub_ = nh.advertise<eyetracking_msgs::RotatedRect>(output_topic_, 10);

  image_transport::ImageTransport it(nh);
  //image_sub_ = it.subscribe("image_rect_color", 1, imageCallback);
  image_sub_ = it.subscribe("image_raw", 1, imageCallback);
  image_pub_ = it.advertise("image_rect_color_pupil", 1);
  ros::spin();
  return 0;
}

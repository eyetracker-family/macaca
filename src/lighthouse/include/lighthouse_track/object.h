#pragma once
#include <opencv2/opencv.hpp>

class Object {
public:

  Object();

  void GetRvec(cv::Matx31d& rv);
  void SetRvec(const cv::Matx31d& rv);

  void GetTvec(cv::Matx31d& tv);
  void SetTvec(const cv::Matx31d& tv);

  void SetId(int);
  int GetId();

private:
  cv::Matx31d rvec_;
  cv::Matx31d tvec_;

  int id_;
};


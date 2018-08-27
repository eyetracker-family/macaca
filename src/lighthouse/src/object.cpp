#include "lighthouse_track/object.h"

Object::Object() :
  id_(0) {
}

void Object::SetTvec( const cv::Matx31d &tv ) {
  tvec_ = tv;
}
void Object::GetTvec( cv::Matx31d &tv ) {
  tv = tvec_;
}
void Object::SetRvec( const cv::Matx31d &rv ) {
  rvec_ = rv;
}
void Object::GetRvec( cv::Matx31d &rv ) {
  rv = rvec_;
}
void Object::SetId(int id) {
  id_ = id;
}
int Object::GetId() {
  return id_;
}

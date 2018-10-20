#include "lighthouse_track/track_object.h"

TrackObject::TrackObject(UsbSerial& usb_serial, const std::vector<cv::Point3d>& vertices, size_t packet_size, size_t serial_data_size) :
  Consumer<LightSensorDataPacket>(data_packet_),
  Productor<LightSensorDataPacket>(data_packet_),
  vertices_(vertices),
  serial_data_size_(serial_data_size),
  usb_serial_(usb_serial) {
     data_packet_ = new RingBuffer<LightSensorDataPacket>(packet_size);
     data_ = new char[serial_data_size];
  }

void TrackObject::GetTrackRvec(cv::Matx31d& rvec) {
  rvec = rvecTrack;
}
void TrackObject::GetTrackTvec(cv::Matx31d& tvec) {
  tvec = tvecTrack;
}
TrackObject::~TrackObject() {
  if (data_packet_) delete data_packet_;
  if (data_) delete data_;
}

bool TrackObject::Product() {
  auto  SearchBeginPos = [](char* buffer, const int size) -> int{
    for (int i = 0; i < size - 1; ++i) {
      if (buffer[i] == '*' && buffer[i + 1] == '#')
        return i;
    }
    return -1;
  };
  auto CheckDataTail = [&]() {
    if (data_[serial_data_size_ - 4] == '#' &&
        data_[serial_data_size_ - 3] == '*' &&
        data_[serial_data_size_ - 2] == '\r' &&
        data_[serial_data_size_ - 1] == '\n')
      return true;
    return false;
  };

  char buffer[serial_data_size_];
  usb_serial_.Read(buffer, serial_data_size_);
  int begin_pos = SearchBeginPos(buffer, serial_data_size_);
  if (begin_pos == -1) {
    return false;
  }
  else {
    memcpy(data_, buffer + begin_pos, serial_data_size_ - begin_pos);
  }
  int ret = usb_serial_.Read(data_ + serial_data_size_ - begin_pos, begin_pos); // read the rest data.
  if (!ret) return ret;

  if (CheckDataTail()) {
    LightSensorDataPacket lsdp;
    memcpy(&lsdp, data_ + 2, sizeof(LightSensorDataPacket));
    if (data_packet_->Write(&lsdp, 1) == 1)
      return true;
    else {
      cerr << "ringbuffer has no free space!" << endl;
      return false;
    }
    return true;
  }
  return false;
}

bool TrackObject::Consume() {
  static auto TimetickToAngle = [](unsigned short timetick)->double {
    return timetick / 8333.33 * PI - PI / 2;
  };
  static auto AngleToTimetick = [](double angle)->unsigned short{
    return (angle + PI / 2) * 8333.33 / PI;
  };

  static auto ProjectionPoint = [](double angle_x, double angle_y)->cv::Point2d {
    double x = tan(angle_x);
    double y = tan(angle_y);
    return cv::Point2d(x, y);
  };
  //auto SolverPnP = [&](){
  LightSensorDataPacket light_sensor_data_packet;
  bool ret = data_packet_->Read(&light_sensor_data_packet, 1);
  if (!ret) {
    return ret;
  } 
  //cout << light_sensor_data_packet.index << endl;
  std::vector<cv::Point2d> image_points;
  std::vector<cv::Point3d> object_points;

  for (int i = 0; i < 36; ++i) {
    if (light_sensor_data_packet.timetick[2*i] == 0 || light_sensor_data_packet.timetick[2*i+1] == 0)
      continue;
     cout << light_sensor_data_packet.timetick[2*i] << "," << light_sensor_data_packet.timetick[2*i+1] << "  "<<endl;
    double angle_x = TimetickToAngle(light_sensor_data_packet.timetick[2*i]);
    double angle_y = TimetickToAngle(light_sensor_data_packet.timetick[2*i + 1]);

    auto img_pt = ProjectionPoint(angle_x, angle_y);
    auto obj_pt = vertices_[i];
    image_points.push_back(img_pt);
    object_points.push_back(obj_pt);
  } 
  //cout << endl << endl;
  if (image_points.size() == object_points.size() && image_points.size() >= 10)
    ret = cv::solvePnP(object_points, image_points, cv::Matx33d::eye(), cv::Mat(), rvecTrack, tvecTrack, false, cv::SOLVEPNP_ITERATIVE);
    //ret = cv::solvePnP(object_points, image_points, cv::Matx33d::eye(), cv::Mat(), rvecTrack, tvecTrack, false, cv::SOLVEPNP_EPNP);
    //ret = cv::solvePnPRansac(object_points, image_points, cv::Matx33d::eye(), cv::Mat(), rvecTrack, tvecTrack, false, 100, 1.0, 100);
  cout << "tvec:\n " << tvecTrack << endl << "rvec:\n" << rvecTrack << endl << endl;
  return ret;
  //};
}

void TrackObject::Start() {
  thread_.push_back(std::thread([&]() {
    Consumer<LightSensorDataPacket>::Start();
  }));
  thread_.push_back(std::thread([&]() {
    Productor<LightSensorDataPacket>::Start();
  }));
}

void TrackObject::Stop() {
  Consumer<LightSensorDataPacket>::Stop();
  Productor<LightSensorDataPacket>::Stop();
}

void TrackObject::Join() {
  for (auto & it : thread_) {
    if (it.joinable())
      it.join();
  }
}

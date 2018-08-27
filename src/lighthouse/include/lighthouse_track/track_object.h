#pragma once
#include "lighthouse_track/object.h"
#include "lighthouse_track/consumer.h"
#include "lighthouse_track/productor.h"
#include "lighthouse_track/serial.h"
#include "lighthouse_track/ringbuffer.h"
#include <thread>

const double PI = 3.1415926;

struct LightSensorDataPacket {
  unsigned int index;
  unsigned short timetick[36 * 2];
};

class TrackObject : public Object, public Consumer<LightSensorDataPacket>, public Productor<LightSensorDataPacket> {
  public:

    TrackObject(UsbSerial& usb_serial, const std::vector<cv::Point3d>& vertices, size_t packet_size = 3, size_t serial_data_size = 154);
    virtual ~TrackObject();

    virtual void Start();
    virtual void Stop();
    virtual bool Product();
    virtual bool Consume();
    virtual void Join();

    void SetVertices(const std::vector<cv::Point3d>& vertices);
    void GetTrackRvec(cv::Matx31d& rvec);
    void GetTrackTvec(cv::Matx31d& tvec);

  private:

    cv::Matx31d rvecTrack;
    cv::Matx31d tvecTrack;

    std::vector<cv::Point3d> vertices_;

    char* data_;
    size_t serial_data_size_;

    UsbSerial& usb_serial_;
    RingBuffer<LightSensorDataPacket>* data_packet_;
    std::vector<std::thread> thread_;

};


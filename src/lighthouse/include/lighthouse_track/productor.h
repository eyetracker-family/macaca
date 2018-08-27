#include "lighthouse_track/ringbuffer.h"
#include <unistd.h>
#include <iostream>

template<typename T>
class Productor {
  public:
    Productor(RingBuffer<T>* rb) :
      ringbuffer_(rb) {
        shall_stop_ = true;
      }

    virtual ~Productor() {
      if (ringbuffer_) delete ringbuffer_;
    }

    virtual bool Product() = 0;

    virtual void Start() {
      shall_stop_ = false;
      while(!shall_stop_) {
        Product();
        usleep(1);
      };
      std::cout << "stoping product" << std::endl;
    }

    virtual void Stop() {
      shall_stop_ = true;
    }

    RingBuffer<T>* ringbuffer_;
  private:
    bool shall_stop_;
};

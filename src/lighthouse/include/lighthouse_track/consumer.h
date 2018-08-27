#pragma once
#include "lighthouse_track/ringbuffer.h"
#include <unistd.h>
#include <thread>
#include <iostream>

template<typename T>
class Consumer {
  public:
    Consumer(RingBuffer<T>* rb) :
      ringbuffer_(rb) {
        shall_stop_ = true;
      }
    virtual ~Consumer() {
      if (ringbuffer_) delete ringbuffer_;
    }

    virtual bool Consume() = 0;

    virtual void Start() {
      shall_stop_ = false;
      while(!shall_stop_) {
        Consume();
        usleep(1);
      };
      std::cout << "stoping consume" << std::endl;
    }

    inline virtual void Stop() {
      shall_stop_ = true;
    }

    RingBuffer<T>* ringbuffer_;
  private:
    bool shall_stop_;
};


#pragma once
#include <algorithm>
#include <mutex>

template<typename T> class RingBuffer {
 public:
  explicit RingBuffer(size_t size)
    : size_(size)
    , begin_(0)
    , end_(0)
    , wrap_(false) {
    buffer_ = new T[size_];
  }
  RingBuffer(const RingBuffer<T> & rb) {
    //this(rb.size_);
    size_ = rb.size_;
    begin_ = rb.begin_;
    end_ = rb.end_;
    memcpy(buffer_, rb.buffer_, sizeof(T) * size_);
  }

  ~RingBuffer() {
    delete[] buffer_;
  }

  size_t Write(const T* data, size_t n) {
    std::lock_guard<std::mutex> lock(mtx_);
    n = std::min<size_t>(n, GetFreeSize());

    if (n == 0) return n;

    const size_t first_chunk = std::min<size_t>(n, size_ - end_);
    memcpy(buffer_ + end_, data, first_chunk * sizeof(T) );
    end_ = (end_ + first_chunk) % size_;

    if (first_chunk < n) {
      const size_t second_chunk = n - first_chunk;
      memcpy(buffer_ + end_, data + first_chunk, second_chunk * sizeof(T));
      end_ = (end_ + second_chunk) % size_;
    }

    if (begin_ == end_) wrap_ = true;
    return n;
  }

  size_t Read(T* dest, size_t n) {
    std::lock_guard<std::mutex> lock(mtx_);
    n = std::min<size_t>(n, GetOccupiedSize());

    if (n == 0) return n;
    if (wrap_) wrap_ = false;

    const size_t first_chunk = std::min<size_t>(n, size_ - begin_);
    memcpy(dest, buffer_ + begin_, first_chunk * sizeof(T));
    begin_ = (begin_ + first_chunk) % size_;

    if (first_chunk < n) {
      const size_t second_chunk = n - first_chunk;
      memcpy(dest + first_chunk, buffer_ + begin_, second_chunk * sizeof(T));
      begin_ = (begin_ + second_chunk) % size_;
    }

    return n;
  }
 private:
  T* buffer_;
  size_t size_;
  size_t begin_;
  size_t end_;
  bool wrap_;
  std::mutex mtx_;
  inline size_t GetFreeSize() {
    size_t occ = 0;
    if (end_ == begin_) {
      occ = wrap_ ? size_ : 0;
    }
    else if (end_ > begin_) {
      occ = end_ - begin_;
    }
    else {
      occ = size_ + end_ - begin_;
    }
    return size_ - occ;
  }
  inline size_t GetOccupiedSize() {
    if (end_ == begin_) {
      return wrap_ ? size_ : 0;
    }
    else if (end_ > begin_) {
      return end_ - begin_;
    }
    else {
      return size_ + end_ - begin_;
    }
  }
};

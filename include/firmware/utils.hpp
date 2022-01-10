#pragma once

#include <cstddef>

inline float clamp(const float value, const float limit) {
  if (value > limit)
    return limit;
  else if (value < -limit)
    return -limit;
  else
    return value;
}

template <class T>
class CircularBuffer {
  T *values_;
  size_t size_;
  size_t iter_;

 public:
  explicit CircularBuffer(size_t size)
      : values_(new T[size]()), size_(size), iter_(0) {}

  T push_back(T val) {
    T tmp = values_[iter_];
    values_[iter_++] = val;
    if (iter_ >= size_) iter_ = 0;
    return tmp;
  }
};

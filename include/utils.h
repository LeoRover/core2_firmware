#ifndef INCLUDE_UTILS_H_
#define INCLUDE_UTILS_H_

#include "hFramework.h"

#include "ros.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt16MultiArray.h"

inline float clamp(float value, float limit) {
  if (value > limit)
    return limit;
  else if (value < -limit)
    return -limit;
  else
    return value;
}

template <class T>
class CircularBuffer {
  T* values_;
  size_t size_;
  size_t iter_;

 public:
  explicit CircularBuffer(uint16_t size)
      : size_(size), iter_(0), values_(new T[size]) {}

  T push_back(T val) {
    T tmp = values_[iter_];
    values_[iter_++] = val;
    if (iter_ >= size_) iter_ = 0;
    return tmp;
  }
};

class ServoWrapper {
  int num_;
  uint16_t current_period_;
  uint16_t servo_period_;
  IServo& servo_;

 public:
  ServoWrapper(int num, IServo& servo, uint16_t period)
      : num_(num), servo_(servo), servo_period_(period) {}

  void angleCallback(const std_msgs::Int16& msg) {
    if (current_period_ != servo_period_) {
      servo_.setPeriod(servo_period_);
      current_period_ = servo_period_;
    }
    servo_.rotAbs(msg.data);
#ifdef DEBUG
    Serial.printf("[servo%dAngleCallback] angle: %d\r\n", num_, msg.data);
#endif
  }

  void pwmCallback(const std_msgs::UInt16MultiArray& msg) {
    if (msg.data_length >= 2) {
      current_period_ = msg.data[0];
      servo_.setPeriod(current_period_);
      servo_.setWidth(msg.data[1]);
#ifdef DEBUG
      Serial.printf("[servo%dPWMCallback] period: %d width: %d\r\n", num_,
                    msg.data[0], msg.data[1]);
    } else {
      Serial.printf(
          "ERROR: [servo%dPWMCallback] data array should have 2 members\r\n",
          num_);
#endif
    }
  }
};

#endif  // INCLUDE_UTILS_H_

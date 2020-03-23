#ifndef LEO_FIRMWARE_INCLUDE_UTILS_H_
#define LEO_FIRMWARE_INCLUDE_UTILS_H_

#include <cstdio>

#include <hFramework.h>

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16MultiArray.h>

#include <leo_firmware/logging.h>

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
  T *values_;
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
  IServo &servo_;

  uint16_t current_period_;
  uint16_t servo_period_;

 public:
  ServoWrapper(int num, IServo &servo) : num_(num), servo_(servo) {}

  void init(ros::NodeHandle *nh) {
    // Default parameters
    int servo_period = 20000;
    int angle_min = -90;
    int angle_max = 90;
    int width_min = 1000;
    int width_max = 2000;

    std::string param_prefix = "core2/servo" + std::to_string(num_) + "/";
    nh->getParam((param_prefix + "period").c_str(), &servo_period);
    nh->getParam((param_prefix + "angle_min").c_str(), &angle_min);
    nh->getParam((param_prefix + "angle_max").c_str(), &angle_max);
    nh->getParam((param_prefix + "width_min").c_str(), &width_min);
    nh->getParam((param_prefix + "width_max").c_str(), &width_max);

    servo_period_ = static_cast<uint16_t>(servo_period);

    servo_.calibrate(
        static_cast<int16_t>(angle_min), static_cast<uint16_t>(width_min),
        static_cast<int16_t>(angle_max), static_cast<uint16_t>(width_max));
  }

  void angleCallback(const std_msgs::Int16 &msg) {
    logDebug("[servo%dAngleCallback] angle: %d", num_, msg.data);
    if (current_period_ != servo_period_) {
      servo_.setPeriod(servo_period_);
      current_period_ = servo_period_;
    }
    servo_.rotAbs(msg.data);
  }

  void pwmCallback(const std_msgs::UInt16MultiArray &msg) {
    if (msg.data_length >= 2) {
      logDebug("[servo%dPWMCallback] period: %d width: %d", num_, msg.data[0],
               msg.data[1]);
      current_period_ = msg.data[0];
      servo_.setPeriod(current_period_);
      servo_.setWidth(msg.data[1]);
    } else {
      logError("[servo%dPWMCallback] data array should have 2 members", num_);
    }
  }
};

#endif  // LEO_FIRMWARE_INCLUDE_UTILS_H_

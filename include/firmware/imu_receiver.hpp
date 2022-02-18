#pragma once

#include <II2C.h>

#include <MPU9250/MPU9250.h>

class ImuReceiver {
 public:
  explicit ImuReceiver(hFramework::II2C &i2c) : mpu_(i2c) {}

  bool init();
  void update();

  float temp;        // temperature
  float ax, ay, az;  // accelerometer data
  float gx, gy, gz;  // gyroscope data

 private:
  MPU9250 mpu_;
  float ares_, gres_;
};
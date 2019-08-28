#ifndef _IMU_H_
#define _IMU_H_

#include "imu/MPU9250.h"

class IMU
{
public:
    IMU(hFramework::hI2C& i2c)
        : mpu_(i2c) {}

    void begin();
    void update();

    float ax, ay, az; // accelerometer data
    float gx, gy, gz; // gyroscope data
    float mx, my, mz; // magnetometer data
    float qx, qy, qz, qw; // quaternion data

private:
    MPU9250 mpu_;
    float ares_, gres_, mres_;
    float magCal_[3];
};

#endif
#ifndef LEO_FIRMWARE_PARAMS_H_
#define LEO_FIRMWARE_PARAMS_H_

#include "hFramework.h"

// The hSens port to which the IMU is connected
// Set to either hSens1 or hSens2
static hFramework::hSensor_i2c &IMU_HSENS = hSens2;

// The hSens port to which the GPS is connected
// Set to either hSens3 or hSens4
static hFramework::hSensor_serial &GPS_HSENS = hSens3;

// The pin which will be used to drive the informative LED on the power switch
// By default it is set to pin1 on hExt port
static hFramework::hGPIO &LED = hExt.pin1;

#endif  // LEO_FIRMWARE_PARAMS_H_

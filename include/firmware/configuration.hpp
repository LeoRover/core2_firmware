#pragma once

#include <peripherals.h>

#include "motor_controller.hpp"
#include "diff_drive_controller.hpp"

const char *const FIRMWARE_VERSION = "2.0.0";

// The hSens port to which the IMU is connected
// Set to either hSens1 or hSens2
static hFramework::hSensor_i2c &IMU_HSENS = hFramework::hSens2;

// The hSens port to which the GPS is connected
// Set to either hSens3 or hSens4
static hFramework::hSensor_serial &GPS_HSENS = hFramework::hSens3;

// The pin which will be used to drive the informative LED on the power switch
// By default it is set to pin1 on hExt port
static hFramework::hGPIO &LED = hFramework::hExt.pin1;

// Number of encoder readings to remember when estimating the wheel velocity
const uint32_t ENCODER_BUFFER_SIZE = 10;

// Number of battery voltage readings to average
const uint32_t BATTERY_BUFFER_SIZE = 3000;

// The period (in milliseconds) between calls to the update() function
const uint16_t UPDATE_PERIOD = 10;

// The periods (in number of calls to the update() function), at which different
// data is publihed on the ROS topics
const uint8_t BATTERY_PUB_PERIOD = 10;
const uint8_t JOINTS_PUB_PERIOD = 5;
const uint8_t ODOM_PUB_PERIOD = 5;
const uint8_t IMU_PUB_PERIOD = 1;

extern MotorController MotA;
extern MotorController MotB;
extern MotorController MotC;
extern MotorController MotD;

const DiffDriveConfiguration DD_CONFIG = {
    .wheel_FL_conf =
        {
            .motor = MotC,
            .encoder_buffer_size = ENCODER_BUFFER_SIZE,
        },
    .wheel_RL_conf =
        {
            .motor = MotD,
            .encoder_buffer_size = ENCODER_BUFFER_SIZE,
        },
    .wheel_FR_conf =
        {
            .motor = MotA,
            .encoder_buffer_size = ENCODER_BUFFER_SIZE,
        },
    .wheel_RR_conf =
        {
            .motor = MotB,
            .encoder_buffer_size = ENCODER_BUFFER_SIZE,
        },
};
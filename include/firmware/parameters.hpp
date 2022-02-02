#pragma once

#include <ros.h>

#include "diff_drive_controller.hpp"

struct Parameters : DiffDriveParams {
  // Override inherited parameters
  Parameters() {
    // Motor
    motor_encoder_resolution = 878.4F;
    motor_pid_p = 0.0F;
    motor_pid_i = 0.005F;
    motor_pid_d = 0.0F;
    motor_pwm_duty_limit = 100.0F;

    // Differential drive
    dd_wheel_radius = 0.0625F;
    dd_wheel_separation = 0.33F;
    dd_angular_velocity_multiplier = 1.91F;
    dd_input_timeout = 500;
  }

  // Servo
  int servo_voltage = 2;
  int servo_period[6] = {20000, 20000, 20000, 20000, 20000, 20000};
  int servo_angle_min[6] = {-90, -90, -90, -90, -90, -90};
  int servo_angle_max[6] = {90, 90, 90, 90, 90, 90};
  int servo_width_min[6] = {1000, 1000, 1000, 1000, 1000, 1000};
  int servo_width_max[6] = {2000, 2000, 2000, 2000, 2000, 2000};

  float battery_min_voltage = 10.0;

  void load(ros::NodeHandle &nh);
};

#pragma once

#include <ros.h>

#include "diff_drive_lib/robot_controller.hpp"

struct Parameters : diff_drive_lib::RobotParams {
  // Override inherited parameters
  Parameters() {
    // Wheel
    wheel_encoder_resolution = 878.4F;
    wheel_pid_p = 0.0F;
    wheel_pid_i = 0.005F;
    wheel_pid_d = 0.0F;
    wheel_pwm_duty_limit = 100.0F;
    wheel_encoder_jump_detection_enabled = true;
    wheel_encoder_jump_threshold = 20000.0F;

    // Differential drive
    robot_wheel_radius = 0.0625F;
    robot_wheel_separation = 0.358F;
    robot_angular_velocity_multiplier = 1.76F;
    robot_input_timeout = 500;
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

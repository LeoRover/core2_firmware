#include <string>

#include "firmware/parameters.hpp"

static constexpr int TIMEOUT = 1000;

Parameters params;

void Parameters::load(ros::NodeHandle &nh) {
  nh.getParam("core2/servo_voltage", &servo_voltage, 1, TIMEOUT);
  for (int i = 1; i <= 6; i++) {
    std::string param_prefix =
        std::string("core2/servo") + static_cast<char>(i + '0') + '/';
    nh.getParam((param_prefix + "period").c_str(), &servo_period[i], 1,
                TIMEOUT);
    nh.getParam((param_prefix + "angle_min").c_str(), &servo_angle_min[i], 1,
                TIMEOUT);
    nh.getParam((param_prefix + "angle_max").c_str(), &servo_angle_max[i], 1,
                TIMEOUT);
    nh.getParam((param_prefix + "width_min").c_str(), &servo_width_min[i], 1,
                TIMEOUT);
    nh.getParam((param_prefix + "width_max").c_str(), &servo_width_max[i], 1,
                TIMEOUT);
  }

  nh.getParam("core2/motors/encoder_resolution", &motor_encoder_resolution, 1,
              TIMEOUT);
  nh.getParam("core2/motors/pid/p", &motor_pid_p, 1, TIMEOUT);
  nh.getParam("core2/motors/pid/i", &motor_pid_i, 1, TIMEOUT);
  nh.getParam("core2/motors/pid/d", &motor_pid_d, 1, TIMEOUT);
  nh.getParam("core2/motors/power_limit", &motor_power_limit, 1, TIMEOUT);

  nh.getParam("core2/diff_drive/wheel_radius", &dd_wheel_radius, 1, TIMEOUT);
  nh.getParam("core2/diff_drive/wheel_separation", &dd_wheel_separation, 1,
              TIMEOUT);
  nh.getParam("core2/diff_drive/angular_velocity_multiplier",
              &dd_angular_velocity_multiplier, 1, TIMEOUT);
  nh.getParam("core2/diff_drive/input_timeout", &dd_input_timeout, 1, TIMEOUT);
  nh.getParam("core2/battery_min_voltage", &battery_min_voltage, 1, TIMEOUT);
}
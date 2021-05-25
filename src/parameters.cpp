#include <string>

#include <leo_firmware/parameters.h>

static constexpr int TIMEOUT = 100;

Parameters params;

void Parameters::load(ros::NodeHandle &nh) {
  char *robot_frame_id_ptr = robot_frame_id;
  nh.getParam("core2/robot_frame_id", &robot_frame_id_ptr, 1, TIMEOUT);
  char *odom_frame_id_ptr = odom_frame_id;
  nh.getParam("core2/odom_frame_id", &odom_frame_id_ptr, 1, TIMEOUT);
  char *imu_frame_id_ptr = imu_frame_id;
  nh.getParam("core2/imu_frame_id", &imu_frame_id_ptr, 1, TIMEOUT);
  char *gps_frame_id_ptr = gps_frame_id;
  nh.getParam("core2/gps_frame_id", &gps_frame_id_ptr, 1, TIMEOUT);

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
  nh.getParam("core2/motors/encoder_pullup", &motor_encoder_pullup, 1, TIMEOUT);
  nh.getParam("core2/motors/max_speed", &motor_max_speed, 1, TIMEOUT);
  nh.getParam("core2/motors/pid/p", &motor_pid_p, 1, TIMEOUT);
  nh.getParam("core2/motors/pid/i", &motor_pid_i, 1, TIMEOUT);
  nh.getParam("core2/motors/pid/d", &motor_pid_d, 1, TIMEOUT);
  nh.getParam("core2/motors/power_limit", &motor_power_limit, 1, TIMEOUT);
  nh.getParam("core2/motors/torque_limit", &motor_torque_limit, 1, TIMEOUT);

  nh.getParam("core2/diff_drive/wheel_radius", &dd_wheel_radius, 1, TIMEOUT);
  nh.getParam("core2/diff_drive/wheel_separation", &dd_wheel_separation, 1,
              TIMEOUT);
  nh.getParam("core2/diff_drive/angular_velocity_multiplier",
              &dd_angular_velocity_multiplier, 1, TIMEOUT);
  nh.getParam("core2/diff_drive/input_timeout", &dd_input_timeout, 1, TIMEOUT);
}
#ifndef INCLUDE_DIFF_DRIVE_CONTROLLER_H_
#define INCLUDE_DIFF_DRIVE_CONTROLLER_H_

#include <vector>

#include "wheel_controller.h"

class DiffDriveController {
 public:
  explicit DiffDriveController(
      const float &wheel_max_speed, const float &pid_p, const float &pid_i,
      const float &pid_d, const uint16_t &power_limit,
      const uint16_t &torque_limit, const bool &encoder_pullup,
      const float &encoder_resolution, const float &wheel_radius,
      const float &robot_width, const uint32_t &input_timeout);
  void start();
  void setSpeed(float linear, float angular);
  std::vector<float> getOdom();
  std::vector<float> getWheelPositions();
  std::vector<float> getWheelVelocities();
  std::vector<float> getWheelEfforts();

 private:
  void updateWheelLoop();
  void updateOdometryLoop();
  void debugLoop();
  void inputWatchdog();

  WheelController *wheel_FL_;
  WheelController *wheel_RL_;
  WheelController *wheel_FR_;
  WheelController *wheel_RR_;

  float lin_vel_;
  float ang_vel_;

  uint32_t last_update_;

  const float wheel_max_speed_;
  const float encoder_resolution_;
  const float wheel_radius_;
  const float robot_width_;
  const uint32_t input_timeout_;
};

#endif  // INCLUDE_DIFF_DRIVE_CONTROLLER_H_

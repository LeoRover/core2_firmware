#ifndef INCLUDE_DIFF_DRIVE_CONTROLLER_H_
#define INCLUDE_DIFF_DRIVE_CONTROLLER_H_

#include <vector>

#include "wheel_controller.h"

class DiffDriveController {
 public:
  explicit DiffDriveController(uint32_t input_timeout = 0);
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

  uint32_t input_timeout_;
  uint32_t last_update_;
};

#endif  // INCLUDE_DIFF_DRIVE_CONTROLLER_H_

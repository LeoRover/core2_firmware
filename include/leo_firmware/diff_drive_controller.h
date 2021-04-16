#ifndef LEO_FIRMWARE_INCLUDE_DIFF_DRIVE_CONTROLLER_H_
#define LEO_FIRMWARE_INCLUDE_DIFF_DRIVE_CONTROLLER_H_

#include <vector>

#include <ros.h>
#include <hMutex.h>

#include <leo_firmware/wheel_controller.h>

class DiffDriveController {
 public:
  void init();
  void setSpeed(float linear, float angular);
  std::vector<float> getOdom();
  std::vector<float> getPose();
  std::vector<float> getWheelPositions();
  std::vector<float> getWheelVelocities();
  std::vector<float> getWheelEfforts();

 private:
  void updateLoop();
  void debugLoop();
  void inputWatchdog();

  hFramework::hMutex mutex_wheel_;
  hFramework::hMutex mutex_odom_;

  WheelController *wheel_FL_;
  WheelController *wheel_RL_;
  WheelController *wheel_FR_;
  WheelController *wheel_RR_;

  float lin_vel_;
  float ang_vel_;
  float lin_pose_;
  float ang_pose_;

  uint64_t last_update_;
};

#endif  // LEO_FIRMWARE_INCLUDE_DIFF_DRIVE_CONTROLLER_H_

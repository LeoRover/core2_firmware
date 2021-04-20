#ifndef LEO_FIRMWARE_INCLUDE_DIFF_DRIVE_CONTROLLER_H_
#define LEO_FIRMWARE_INCLUDE_DIFF_DRIVE_CONTROLLER_H_

#include <vector>

#include <ros.h>
#include <hMutex.h>

#include <leo_firmware/wheel_controller.h>

struct Odom {
  float vel_lin;
  float vel_ang;
  float pose_x;
  float pose_y;
  float pose_yaw;
};

class DiffDriveController {
 public:
  void init();
  void setSpeed(const float linear, const float angular);
  Odom getOdom();
  void resetOdom();
  void updateWheelStates();

  double positions[4];
  double velocities[4];
  double efforts[4];

 private:
  void controllerLoop();
  void inputWatchdog();

  hFramework::hMutex mutex_wheel_;
  hFramework::hMutex mutex_odom_;

  WheelController *wheel_FL_;
  WheelController *wheel_RL_;
  WheelController *wheel_FR_;
  WheelController *wheel_RR_;

  Odom odom_;

  uint64_t last_update_;
};

#endif  // LEO_FIRMWARE_INCLUDE_DIFF_DRIVE_CONTROLLER_H_

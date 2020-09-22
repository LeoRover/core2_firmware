#ifndef LEO_FIRMWARE_INCLUDE_DIFF_DRIVE_CONTROLLER_H_
#define LEO_FIRMWARE_INCLUDE_DIFF_DRIVE_CONTROLLER_H_

#include <vector>

#include <ros.h>

#include <leo_firmware/wheel_controller.h>

class DiffDriveController {
 public:
  void init(ros::NodeHandle *nh);
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

  uint64_t last_update_;
  uint64_t input_timeout_;

  // Default parameters
  float encoder_resolution_ = 878.4;
  float wheel_radius_ = 0.0625;
  float wheel_separation_ = 0.33;
  float angular_velocity_multiplier_ = 1.91;
};

#endif  // LEO_FIRMWARE_INCLUDE_DIFF_DRIVE_CONTROLLER_H_

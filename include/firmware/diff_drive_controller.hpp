#pragma once

#include <vector>

#include <ros.h>

#include <leo_msgs/WheelOdom.h>
#include <leo_msgs/WheelStates.h>

#include "firmware/diff_drive_controller.hpp"
#include "firmware/motor_controller.hpp"
#include "firmware/wheel_controller.hpp"

struct Odom {
  float vel_lin;
  float vel_ang;
  float pose_x;
  float pose_y;
  float pose_yaw;
};

struct DiffDriveConfiguration {
  WheelConfiguration wheel_FL_conf;
  WheelConfiguration wheel_RL_conf;
  WheelConfiguration wheel_FR_conf;
  WheelConfiguration wheel_RR_conf;
};

class DiffDriveController {
 public:
  DiffDriveController(const DiffDriveConfiguration& dd_conf);

  /**
   * Initialize the Diff Drive Controller.
   * Should be called after all ROS parameters are loaded.
   * Initializes all Wheel Controllers.
   */
  void init();

  /**
   * Set the target speed of the robot.
   * @param linear The linear speed of the robot in m/s
   * @param angular The angular speed of the robot in rad/s
   */
  void setSpeed(float linear, float angular);

  /**
   * Get the current odometry.
   */
  leo_msgs::WheelOdom getOdom();

  /**
   * Reset the odometry position.
   */
  void resetOdom();

  /**
   * Retrieve the wheel states and populate the WheelStates structure fields
   * with the new values.
   * @param wheel_states A reference to the structure to modify
   */
  void updateWheelStates(leo_msgs::WheelStates& wheel_states);

  /**
   * Perform an update routine.
   * @param dt_ms Time elapsed since the last call to update function
   */
  void update(uint32_t dt_ms);

  void enable();
  void disable();

  double positions[4];
  double velocities[4];
  double efforts[4];

  WheelController wheel_FL;
  WheelController wheel_RL;
  WheelController wheel_FR;
  WheelController wheel_RR;

 private:
  leo_msgs::WheelOdom odom_;
  bool enabled_ = false;
  uint32_t last_command_time_;
};

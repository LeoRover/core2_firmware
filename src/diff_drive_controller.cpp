#include <cstdio>

#include <hSystem.h>
#include <peripherals.h>

#include <leo_firmware/diff_drive_controller.h>
#include <leo_firmware/logging.h>
#include <leo_firmware/parameters.h>
#include <leo_firmware/utils.h>

using hFramework::sys;

static const float PI = 3.14159265358979323846;

void DiffDriveController::init() {
  wheel_FL_ = new WheelController(hFramework::hMotC, true);
  wheel_RL_ = new WheelController(hFramework::hMotD, true);
  wheel_FR_ = new WheelController(hFramework::hMotA, false);
  wheel_RR_ = new WheelController(hFramework::hMotB, false);

  sys.taskCreate(std::bind(&DiffDriveController::updateLoop, this));
  if (params.dd_input_timeout > 0.0) {
    last_update_ = sys.getRefTime();
    sys.taskCreate(std::bind(&DiffDriveController::inputWatchdog, this));
  }
}

void DiffDriveController::setSpeed(float linear, float angular) {
  angular *= params.dd_angular_velocity_multiplier;
  float wheel_L_lin_vel = linear - (angular * params.dd_wheel_separation / 2);
  float wheel_R_lin_vel = linear + (angular * params.dd_wheel_separation / 2);
  float wheel_L_ang_vel = wheel_L_lin_vel / params.dd_wheel_radius;
  float wheel_R_ang_vel = wheel_R_lin_vel / params.dd_wheel_radius;
  float enc_L_speed = params.motor_encoder_resolution * wheel_L_ang_vel / (2 * PI);
  float enc_R_speed = params.motor_encoder_resolution * wheel_R_ang_vel / (2 * PI);

  wheel_FL_->setSpeed(enc_L_speed);
  wheel_RL_->setSpeed(enc_L_speed);
  wheel_FR_->setSpeed(enc_R_speed);
  wheel_RR_->setSpeed(enc_R_speed);

  if (params.dd_input_timeout > 0.0) last_update_ = sys.getRefTime();
}

std::vector<float> DiffDriveController::getOdom() {
  hFramework::hMutexGuard m(mutex_odom_);
  std::vector<float> odom(2);
  odom[0] = lin_vel_;
  odom[1] = ang_vel_;
  return odom;
}

std::vector<float> DiffDriveController::getPose() {
  hFramework::hMutexGuard m(mutex_odom_);
  std::vector<float> pose(2);
  pose[0] = lin_pose_;
  pose[1] = ang_pose_;
  return pose;
}

std::vector<float> DiffDriveController::getWheelPositions() {
  hFramework::hMutexGuard m(mutex_wheel_);
  std::vector<float> positions(4);
  positions[0] = 2 * PI * wheel_FL_->getDistance() / params.motor_encoder_resolution;
  positions[1] = 2 * PI * wheel_RL_->getDistance() / params.motor_encoder_resolution;
  positions[2] = 2 * PI * wheel_FR_->getDistance() / params.motor_encoder_resolution;
  positions[3] = 2 * PI * wheel_RR_->getDistance() / params.motor_encoder_resolution;
  return positions;
}

std::vector<float> DiffDriveController::getWheelVelocities() {
  hFramework::hMutexGuard m(mutex_wheel_);
  std::vector<float> velocities(4);
  velocities[0] = 2 * PI * wheel_FL_->getSpeed() / params.motor_encoder_resolution;
  velocities[1] = 2 * PI * wheel_RL_->getSpeed() / params.motor_encoder_resolution;
  velocities[2] = 2 * PI * wheel_FR_->getSpeed() / params.motor_encoder_resolution;
  velocities[3] = 2 * PI * wheel_RR_->getSpeed() / params.motor_encoder_resolution;
  return velocities;
}

std::vector<float> DiffDriveController::getWheelEfforts() {
  hFramework::hMutexGuard m(mutex_wheel_);
  std::vector<float> efforts(4);
  efforts[0] = wheel_FL_->getPower() * 0.1;
  efforts[1] = wheel_RL_->getPower() * 0.1;
  efforts[2] = wheel_FR_->getPower() * 0.1;
  efforts[3] = wheel_RR_->getPower() * 0.1;
  return efforts;
}

void DiffDriveController::updateLoop() {
  uint32_t t = sys.getRefTime();
  uint32_t dt = 10;
  while (true) {
    mutex_wheel_.lock();
    {
      wheel_FL_->update(dt);
      wheel_RL_->update(dt);
      wheel_FR_->update(dt);
      wheel_RR_->update(dt);
    }
    mutex_wheel_.unlock();

    // speed in ticks/sec
    float FL_speed = wheel_FL_->getSpeed();
    float RL_speed = wheel_RL_->getSpeed();
    float FR_speed = wheel_FR_->getSpeed();
    float RR_speed = wheel_RR_->getSpeed();

    float L_speed = (FL_speed + RL_speed) / 2.0;
    float R_speed = (FR_speed + RR_speed) / 2.0;

    // velocity in radians per second
    float L_ang_vel = 2 * PI * L_speed / params.motor_encoder_resolution;
    float R_ang_vel = 2 * PI * R_speed / params.motor_encoder_resolution;

    // velocity in meters per second
    float L_lin_vel = L_ang_vel * params.dd_wheel_radius;
    float R_lin_vel = R_ang_vel * params.dd_wheel_radius;

    mutex_odom_.lock();
    {
      // linear (m/s) and angular (r/s) velocities of the robot
      lin_vel_ = (L_lin_vel + R_lin_vel) / 2;
      ang_vel_ = (R_lin_vel - L_lin_vel) / params.dd_wheel_separation;

      ang_vel_ /= params.dd_angular_velocity_multiplier;
    }
    mutex_odom_.unlock();

    sys.delaySync(t, dt);
  }
}

void DiffDriveController::inputWatchdog() {
  while (true) {
    while (sys.getRefTime() < last_update_ + params.dd_input_timeout)
      sys.delay(last_update_ + params.dd_input_timeout - sys.getRefTime() + 1);

    setSpeed(0.0, 0.0);
  }
}

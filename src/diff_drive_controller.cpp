#include <cstdio>

#include <hFramework.h>

#include <leo_firmware/diff_drive_controller.h>
#include <leo_firmware/logging.h>
#include <leo_firmware/utils.h>

void DiffDriveController::init(ros::NodeHandle *nh) {
  // Default parameters
  float wheel_max_speed = 800.0;
  float pid_p = 0.0;
  float pid_i = 0.005;
  float pid_d = 0.0;
  float power_limit = 1000.0;
  float torque_limit = 800.0;
  int encoder_pullup = 1;
  int timeout = 500;

  nh->getParam("core2/motors/encoder_resolution", &encoder_resolution_);
  nh->getParam("core2/motors/encoder_pullup", &encoder_pullup);
  nh->getParam("core2/motors/max_speed", &wheel_max_speed);
  nh->getParam("core2/motors/pid/p", &pid_p);
  nh->getParam("core2/motors/pid/i", &pid_i);
  nh->getParam("core2/motors/pid/d", &pid_d);
  nh->getParam("core2/motors/power_limit", &power_limit);
  nh->getParam("core2/motors/torque_limit", &torque_limit);

  nh->getParam("core2/diff_drive/wheel_radius", &wheel_radius_);
  nh->getParam("core2/diff_drive/wheel_separation", &wheel_separation_);
  nh->getParam("core2/diff_drive/angular_velocity_multiplier",
               &angular_velocity_multiplier_);
  nh->getParam("core2/diff_drive/input_timeout", &timeout);

  input_timeout_ = static_cast<uint64_t>(timeout);
  bool pullup = encoder_pullup == 1 ? true : false;

  wheel_FL_ = new WheelController(hMotC, 1, wheel_max_speed, pid_p, pid_i,
                                  pid_d, power_limit, torque_limit, pullup);
  wheel_RL_ = new WheelController(hMotD, 1, wheel_max_speed, pid_p, pid_i,
                                  pid_d, power_limit, torque_limit, pullup);
  wheel_FR_ = new WheelController(hMotA, 0, wheel_max_speed, pid_p, pid_i,
                                  pid_d, power_limit, torque_limit, pullup);
  wheel_RR_ = new WheelController(hMotB, 0, wheel_max_speed, pid_p, pid_i,
                                  pid_d, power_limit, torque_limit, pullup);
}

void DiffDriveController::start() {
  sys.taskCreate(std::bind(&DiffDriveController::updateWheelLoop, this));
  sys.taskCreate(std::bind(&DiffDriveController::updateOdometryLoop, this));
  if (input_timeout_ > 0.0) {
    last_update_ = sys.getRefTime();
    sys.taskCreate(std::bind(&DiffDriveController::inputWatchdog, this));
  }
  sys.taskCreate(std::bind(&DiffDriveController::debugLoop, this));
}

void DiffDriveController::setSpeed(float linear, float angular) {
  angular *= angular_velocity_multiplier_;
  float wheel_L_lin_vel = linear - (angular * wheel_separation_ / 2);
  float wheel_R_lin_vel = linear + (angular * wheel_separation_ / 2);
  float wheel_L_ang_vel = wheel_L_lin_vel / wheel_radius_;
  float wheel_R_ang_vel = wheel_R_lin_vel / wheel_radius_;
  float enc_L_speed = encoder_resolution_ * wheel_L_ang_vel / (2 * M_PI);
  float enc_R_speed = encoder_resolution_ * wheel_R_ang_vel / (2 * M_PI);

  wheel_FL_->setSpeed(enc_L_speed);
  wheel_RL_->setSpeed(enc_L_speed);
  wheel_FR_->setSpeed(enc_R_speed);
  wheel_RR_->setSpeed(enc_R_speed);

  if (input_timeout_ > 0.0) last_update_ = sys.getRefTime();
}

std::vector<float> DiffDriveController::getOdom() {
  std::vector<float> odom;
  odom.push_back(lin_vel_);
  odom.push_back(ang_vel_);
  return odom;
}

std::vector<float> DiffDriveController::getWheelPositions() {
  std::vector<float> positions(4);
  positions[0] = 2 * M_PI * wheel_FL_->getDistance() / encoder_resolution_;
  positions[1] = 2 * M_PI * wheel_RL_->getDistance() / encoder_resolution_;
  positions[2] = 2 * M_PI * wheel_FR_->getDistance() / encoder_resolution_;
  positions[3] = 2 * M_PI * wheel_RR_->getDistance() / encoder_resolution_;
  return positions;
}

std::vector<float> DiffDriveController::getWheelVelocities() {
  std::vector<float> velocities(4);
  velocities[0] = 2 * M_PI * wheel_FL_->getSpeed() / encoder_resolution_;
  velocities[1] = 2 * M_PI * wheel_RL_->getSpeed() / encoder_resolution_;
  velocities[2] = 2 * M_PI * wheel_FR_->getSpeed() / encoder_resolution_;
  velocities[3] = 2 * M_PI * wheel_RR_->getSpeed() / encoder_resolution_;
  return velocities;
}

std::vector<float> DiffDriveController::getWheelEfforts() {
  std::vector<float> efforts(4);
  efforts[0] = wheel_FL_->getPower() * 0.1;
  efforts[1] = wheel_RL_->getPower() * 0.1;
  efforts[2] = wheel_FR_->getPower() * 0.1;
  efforts[3] = wheel_RR_->getPower() * 0.1;
  return efforts;
}

void DiffDriveController::updateWheelLoop() {
  uint32_t t = sys.getRefTime();
  uint32_t dt = 10;
  while (true) {
    wheel_FL_->update(dt);
    wheel_RL_->update(dt);
    wheel_FR_->update(dt);
    wheel_RR_->update(dt);
    sys.delaySync(t, dt);
  }
}

void DiffDriveController::updateOdometryLoop() {
  uint32_t t = sys.getRefTime();
  uint32_t dt = 10;

  while (true) {
    // speed in ticks/sec
    float FL_speed = wheel_FL_->getSpeed();
    float RL_speed = wheel_RL_->getSpeed();
    float FR_speed = wheel_FR_->getSpeed();
    float RR_speed = wheel_RR_->getSpeed();

    float L_speed = (FL_speed + RL_speed) / 2.0;
    float R_speed = (FR_speed + RR_speed) / 2.0;

    // velocity in radians per second
    float L_ang_vel = 2 * M_PI * L_speed / encoder_resolution_;
    float R_ang_vel = 2 * M_PI * R_speed / encoder_resolution_;

    // velocity in meters per second
    float L_lin_vel = L_ang_vel * wheel_radius_;
    float R_lin_vel = R_ang_vel * wheel_radius_;

    // linear (m/s) and angular (r/s) velocities of the robot
    lin_vel_ = (L_lin_vel + R_lin_vel) / 2;
    ang_vel_ = (R_lin_vel - L_lin_vel) / wheel_separation_;

    ang_vel_ /= angular_velocity_multiplier_;

    sys.delaySync(t, dt);
  }
}

void DiffDriveController::debugLoop() {
  uint32_t t = sys.getRefTime();
  uint32_t dt = 100;

  while (true) {
    logDebug("[Motor power] FL: %d RL: %d FR: %d RR: %d", wheel_FL_->getPower(),
             wheel_RL_->getPower(), wheel_FR_->getPower(),
             wheel_RR_->getPower());
    logDebug("[Motor speed] FL: %f RL: %f FR: %f RR: %f", wheel_FL_->getSpeed(),
             wheel_RL_->getSpeed(), wheel_FR_->getSpeed(),
             wheel_RR_->getSpeed());
    sys.delaySync(t, dt);
  }
}

void DiffDriveController::inputWatchdog() {
  while (true) {
    while (sys.getRefTime() < last_update_ + input_timeout_)
      sys.delay(last_update_ + input_timeout_ - sys.getRefTime() + 1);

    setSpeed(0.0, 0.0);
  }
}

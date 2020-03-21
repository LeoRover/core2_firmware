#include "hFramework.h"

#include "diff_drive_controller.h"
#include "params.h"
#include "utils.h"

DiffDriveController::DiffDriveController(uint32_t input_timeout)
    : input_timeout_(input_timeout) {
  wheel_FL_ =
      new WheelController(hMotC, 1, WHEEL_MAX_SPEED, PID_P, PID_I, PID_D,
                          POWER_LIMIT, TORQUE_LIMIT, ENCODER_PULLUP);
  wheel_RL_ =
      new WheelController(hMotD, 1, WHEEL_MAX_SPEED, PID_P, PID_I, PID_D,
                          POWER_LIMIT, TORQUE_LIMIT, ENCODER_PULLUP);
  wheel_FR_ =
      new WheelController(hMotA, 0, WHEEL_MAX_SPEED, PID_P, PID_I, PID_D,
                          POWER_LIMIT, TORQUE_LIMIT, ENCODER_PULLUP);
  wheel_RR_ =
      new WheelController(hMotB, 0, WHEEL_MAX_SPEED, PID_P, PID_I, PID_D,
                          POWER_LIMIT, TORQUE_LIMIT, ENCODER_PULLUP);
}

void DiffDriveController::start() {
  sys.taskCreate(std::bind(&DiffDriveController::updateWheelLoop, this));
  sys.taskCreate(std::bind(&DiffDriveController::updateOdometryLoop, this));
  if (input_timeout_ > 0.0) {
    last_update_ = sys.getRefTime();
    sys.taskCreate(std::bind(&DiffDriveController::inputWatchdog, this));
  }
#ifdef DEBUG
  sys.taskCreate(std::bind(&DiffDriveController::debugLoop, this));
#endif
}

void DiffDriveController::setSpeed(float linear, float angular) {
  float wheel_L_lin_vel = linear - (angular * ROBOT_WIDTH / 2);
  float wheel_R_lin_vel = linear + (angular * ROBOT_WIDTH / 2);
  float wheel_L_ang_vel = wheel_L_lin_vel / WHEEL_RADIUS;
  float wheel_R_ang_vel = wheel_R_lin_vel / WHEEL_RADIUS;
  float enc_L_speed =
      clamp(ENCODER_RESOLUTION * wheel_L_ang_vel / (2 * M_PI), WHEEL_MAX_SPEED);
  float enc_R_speed =
      clamp(ENCODER_RESOLUTION * wheel_R_ang_vel / (2 * M_PI), WHEEL_MAX_SPEED);

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
  positions[0] = 2 * M_PI * wheel_FL_->getDistance() / ENCODER_RESOLUTION;
  positions[1] = 2 * M_PI * wheel_RL_->getDistance() / ENCODER_RESOLUTION;
  positions[2] = 2 * M_PI * wheel_FR_->getDistance() / ENCODER_RESOLUTION;
  positions[3] = 2 * M_PI * wheel_RR_->getDistance() / ENCODER_RESOLUTION;
  return positions;
}

std::vector<float> DiffDriveController::getWheelVelocities() {
  std::vector<float> velocities(4);
  velocities[0] = 2 * M_PI * wheel_FL_->getSpeed() / ENCODER_RESOLUTION;
  velocities[1] = 2 * M_PI * wheel_RL_->getSpeed() / ENCODER_RESOLUTION;
  velocities[2] = 2 * M_PI * wheel_FR_->getSpeed() / ENCODER_RESOLUTION;
  velocities[3] = 2 * M_PI * wheel_RR_->getSpeed() / ENCODER_RESOLUTION;
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
    float L_ang_vel = 2 * M_PI * L_speed / ENCODER_RESOLUTION;
    float R_ang_vel = 2 * M_PI * R_speed / ENCODER_RESOLUTION;

    // velocity in meters per second
    float L_lin_vel = L_ang_vel * WHEEL_RADIUS;
    float R_lin_vel = R_ang_vel * WHEEL_RADIUS;

    // linear (m/s) and angular (r/s) velocities of the robot
    lin_vel_ = (L_lin_vel + R_lin_vel) / 2;
    ang_vel_ = (R_lin_vel - L_lin_vel) / ROBOT_WIDTH;

    sys.delaySync(t, dt);
  }
}

void DiffDriveController::debugLoop() {
  uint32_t t = sys.getRefTime();
  uint32_t dt = 100;

  while (true) {
    Serial.printf("Motor powers: %d %d %d %d\r\n", wheel_FL_->getPower(),
                  wheel_RL_->getPower(), wheel_FR_->getPower(),
                  wheel_RR_->getPower());
    Serial.printf("Motor speeds: %f %f %f %f\r\n", wheel_FL_->getSpeed(),
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

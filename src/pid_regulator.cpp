#include "firmware/pid_regulator.hpp"
#include "firmware/utils.hpp"

PIDRegulator::PIDRegulator(const float Kp, const float Ki, const float Kd,
                           const float range)
    : Kp_(Kp), Ki_(Ki), Kd_(Kd), range_(range) {}

void PIDRegulator::setCoeffs(const float Kp, const float Ki, const float Kd) {
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;
}

void PIDRegulator::setRange(const float range) { range_ = range; }

void PIDRegulator::reset() {
  isum_ = 0;
  last_error_ = 0;
  has_last_error_ = false;
}

float PIDRegulator::update(const float error, const uint16_t dt_ms) {
  float cur_err;
  if (has_last_error_) {
    cur_err = error - last_error_;
  } else {
    cur_err = 0;
    has_last_error_ = true;
  }
  last_error_ = error;

  isum_ += Ki_ * error * static_cast<float>(dt_ms);

  // Anti-windup
  isum_ = clamp(isum_, range_);

  float val = Kp_ * error + isum_ + Kd_ * cur_err / static_cast<float>(dt_ms);
  val = clamp(val, range_);

  return val;
}

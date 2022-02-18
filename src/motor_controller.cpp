#include "firmware/motor_controller.hpp"

inline float clamp(const float value, const float limit) {
  if (value > limit)
    return limit;
  else if (value < -limit)
    return -limit;
  else
    return value;
}

void MotorController::init() {
  motor_.setEncoderPu();
  if (reverse_polarity_) {
    motor_.setMotorPolarity(Polarity::Reversed);
    motor_.setEncoderPolarity(Polarity::Normal);
  } else {
    motor_.setMotorPolarity(Polarity::Normal);
    motor_.setEncoderPolarity(Polarity::Reversed);
  }
}

void MotorController::setPWMDutyCycle(float pwm_duty) {
  pwm_duty_ = clamp(pwm_duty, 100.0F);

  int16_t power = static_cast<int16_t>((pwm_duty_ / 100.0F) * 1000.0);

  motor_.setPower(power);
}

float MotorController::getPWMDutyCycle() {
  return pwm_duty_;
}

int32_t MotorController::getEncoderCnt() {
  return motor_.getEncoderCnt();
}

void MotorController::resetEncoderCnt() {
  motor_.resetEncoderCnt();
}

float MotorController::getWindingCurrent() {
  return 0.0;
}

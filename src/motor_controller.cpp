#include "firmware/motor_controller.hpp"
#include "firmware/configuration.hpp"

void MotorController::init() { motor_.setEncoderPu(); }

void MotorController::setPWMDutyCycle(float pwm_duty) {
  pwm_duty_ = clamp(pwm_duty, 100.0F);

  int16_t power = static_cast<int16_t>((pwm_duty_ / 100.0F) * 1000.0);

  motor_.setPower(power);
}

float MotorController::getPWMDutyCycle() { return pwm_duty_; }

int32_t MotorController::getEncoderCnt() { return motor_.getEncoderCnt(); }

void MotorController::resetEncoderCnt() { motor_.resetEncoderCnt(); }

float MotorController::getWindingCurrent() { return 0.0; }

void MotorController::setMotorPolarity(Polarity polarity) {
  motor_.setMotorPolarity(polarity);
}

void MotorController::setEncoderPolarity(Polarity polarity) {
  motor_.setEncoderPolarity(polarity);
}
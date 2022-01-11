#include <algorithm>

#include "firmware/configuration.hpp"
#include "firmware/motor_controller.hpp"
#include "firmware/parameters.hpp"
#include "firmware/utils.hpp"
#include "firmware/wheel_controller.hpp"

static constexpr float PI = 3.141592653F;

WheelController::WheelController(const WheelConfiguration& wheel_conf)
    : motor(wheel_conf.motor_conf),
      reverse_polarity_(wheel_conf.reverse_polarity),
      encoder_buffer_(ENCODER_BUFFER_SIZE) {}

void WheelController::init() {
  v_reg_.setCoeffs(params.motor_pid_p, params.motor_pid_i, params.motor_pid_d);
  v_reg_.setRange(std::min(1000.0F, params.motor_power_limit));
  motor.init();
  if (reverse_polarity_) {
    motor.setMotorPolarity(Polarity::Reversed);
    motor.setEncoderPolarity(Polarity::Reversed);
  }
  motor.resetEncoderCnt();
}

void WheelController::update(const uint32_t dt_ms) {
  int32_t ticks_prev = ticks_now_;
  ticks_now_ = motor.getEncoderCnt();

  int32_t new_ticks = ticks_now_ - ticks_prev;

  std::pair<int32_t, uint32_t> encoder_old =
      encoder_buffer_.push_back(std::pair<int32_t, uint32_t>(new_ticks, dt_ms));

  ticks_sum_ += new_ticks;
  dt_sum_ += dt_ms;

  ticks_sum_ -= encoder_old.first;
  dt_sum_ -= encoder_old.second;

  v_now_ = static_cast<float>(ticks_sum_) / (dt_sum_ * 0.001F);

  if (enabled_) {
    float pwm_duty;
    if (v_now_ == 0.0F && v_target_ == 0.0F) {
      v_reg_.reset();
      pwm_duty = 0.0F;
    } else {
      float v_err = v_target_ - v_now_;
      pwm_duty = v_reg_.update(v_err, dt_ms) / 10.0F;
    }
    motor.setPWMDutyCycle(pwm_duty);
  }
}

void WheelController::setTargetVelocity(const float speed) {
  v_target_ = (speed / (2.0F * PI)) * params.motor_encoder_resolution;
}

float WheelController::getVelocity() {
  return (v_now_ / params.motor_encoder_resolution) * (2.0F * PI);
}

float WheelController::getTorque() { return 0.0; }

float WheelController::getDistance() {
  return (ticks_now_ / params.motor_encoder_resolution) * (2.0F * PI);
}

void WheelController::resetDistance() {
  motor.resetEncoderCnt();
  ticks_now_ = 0;
}

void WheelController::enable() {
  if (!enabled_) {
    v_reg_.reset();
    enabled_ = true;
  }
}

void WheelController::disable() {
  if (enabled_) {
    enabled_ = false;
    motor.setPWMDutyCycle(0.0F);
  }
}

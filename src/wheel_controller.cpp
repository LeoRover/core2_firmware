#include <leo_firmware/utils.h>
#include <leo_firmware/wheel_controller.h>

WheelController::WheelController(hMotor &motor, const bool polarity,
                                 const float max_speed, const float kp,
                                 const float ki, const float kd,
                                 const uint16_t power_limit,
                                 const uint16_t torque_limit,
                                 const bool encoder_pullup)
    : motor_(motor),
      polarity_(polarity),
      max_speed_(max_speed),
      power_limit_(power_limit),
      torque_limit_(torque_limit),
      turned_on_(true),
      ticks_now_(0),
      ticks_sum_(0),
      dt_sum_(0),
      v_now_(0.0),
      v_target_(0.0),
      encoder_buffer_(encoder_buffer_size_) {
  v_reg_.setScale(1);
  v_reg_.setRange(-v_range_, v_range_);
  v_reg_.setIRange(-v_range_, v_range_);
  v_reg_.setCoeffs(kp, ki, kd);

  if (polarity_) {
    motor_.setMotorPolarity(Polarity::Reversed);
    motor_.setEncoderPolarity(Polarity::Reversed);
  }

  if (encoder_pullup)
    motor_.setEncoderPu();
  else
    motor_.setEncoderPd();

  motor_.setPowerLimit(power_limit);
  motor_.resetEncoderCnt();
}

void WheelController::update(uint32_t dt) {
  int32_t ticks_prev = ticks_now_;
  ticks_now_ = motor_.getEncoderCnt();

  int32_t new_ticks = ticks_now_ - ticks_prev;

  std::pair<int32_t, uint32_t> encoder_old =
      encoder_buffer_.push_back(std::pair<int32_t, uint32_t>(new_ticks, dt));

  ticks_sum_ += new_ticks;
  dt_sum_ += dt;

  ticks_sum_ -= encoder_old.first;
  dt_sum_ -= encoder_old.second;

  v_now_ = static_cast<float>(ticks_sum_) / (dt_sum_ * 0.001);

  float v_err = v_now_ - v_target_;
  float pid_out = v_reg_.update(v_err, dt);

  float est_power = (std::abs(v_now_) / max_speed_) * 1000.0;
  float max_power = std::min(est_power + static_cast<float>(torque_limit_),
                             static_cast<float>(1000.0));

  if (turned_on_ == true) {
    if (v_now_ == 0.0 && v_target_ == 0.0) {
      v_reg_.reset();
      power_ = 0;
    } else {
      float power_limited = clamp(pid_out, max_power);
      power_ = static_cast<int16_t>(power_limited);
    }
    motor_.setPower(power_);
  }
}

void WheelController::setSpeed(float speed) {
  v_target_ = clamp(speed, max_speed_);
}

float WheelController::getSpeed() { return v_now_; }

int16_t WheelController::getPower() { return power_; }

int32_t WheelController::getDistance() { return ticks_now_; }

void WheelController::resetDistance() { motor_.resetEncoderCnt(); }

void WheelController::reset() {
  motor_.resetEncoderCnt();
  v_reg_.reset();
  v_now_ = 0;
  v_target_ = 0;
  motor_.setPower(0);
}

void WheelController::turnOff() {
  turned_on_ = false;
  motor_.setPower(0);
}
void WheelController::turnOn() { turned_on_ = true; }

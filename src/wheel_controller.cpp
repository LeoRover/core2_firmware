#include <leo_firmware/parameters.h>
#include <leo_firmware/utils.h>
#include <leo_firmware/wheel_controller.h>

WheelController::WheelController(hFramework::hMotor &motor, const bool polarity)
    : motor_(motor),
      polarity_(polarity),
      turned_on_(true),
      ticks_now_(0),
      ticks_offset_(0),
      ticks_sum_(0),
      dt_sum_(0),
      v_now_(0.0),
      v_target_(0.0),
      encoder_buffer_(ENCODER_BUFFER_SIZE) {
  v_reg_.setScale(1);
  v_reg_.setRange(-V_RANGE, V_RANGE);
  v_reg_.setIRange(-V_RANGE, V_RANGE);
  v_reg_.setCoeffs(params.motor_pid_p, params.motor_pid_i, params.motor_pid_d);

  if (polarity_) {
    motor_.setMotorPolarity(Polarity::Reversed);
    motor_.setEncoderPolarity(Polarity::Reversed);
  }

  if (params.motor_encoder_pullup)
    motor_.setEncoderPu();
  else
    motor_.setEncoderPd();

  motor_.setPowerLimit(params.motor_power_limit);
  motor_.resetEncoderCnt();
}

void WheelController::update(uint32_t dt) {
  int32_t ticks_prev = ticks_now_;
  ticks_now_ = motor_.getEncoderCnt() - ticks_offset_;

  int32_t new_ticks = ticks_now_ - ticks_prev;

  float ins_vel = static_cast<float>(std::abs(new_ticks)) / (dt * 0.001);
  if (ins_vel > WHEEL_VELOCITY_REJECTION_THRESHOLD * params.motor_max_speed) {
    ticks_offset_ += new_ticks;
    ticks_now_ -= new_ticks;
    new_ticks = 0;
  }

  std::pair<int32_t, uint32_t> encoder_old =
      encoder_buffer_.push_back(std::pair<int32_t, uint32_t>(new_ticks, dt));

  ticks_sum_ += new_ticks;
  dt_sum_ += dt;

  ticks_sum_ -= encoder_old.first;
  dt_sum_ -= encoder_old.second;

  v_now_ = static_cast<float>(ticks_sum_) / (dt_sum_ * 0.001);

  float v_err = v_now_ - v_target_;
  float pid_out = v_reg_.update(v_err, dt);

  float est_power = (std::abs(v_now_) / params.motor_max_speed) * 1000.0;
  float max_power = std::min(est_power + static_cast<float>(params.motor_torque_limit),
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
  v_target_ = clamp(speed, params.motor_max_speed);
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
  ticks_now_ = 0;
  ticks_offset_ = 0;
}

void WheelController::turnOff() {
  turned_on_ = false;
  motor_.setPower(0);
}
void WheelController::turnOn() { turned_on_ = true; }

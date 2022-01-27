#pragma once

#include <hMotor.h>

#include "motor_controller_interface.hpp"

class Core2MotorController : public MotorControllerInterface {
 public:
  explicit Core2MotorController(hFramework::hMotor& motor,
                                const bool reverse_polarity)
      : motor_(motor), reverse_polarity_(reverse_polarity) {}

  void init() override;
  void setPWMDutyCycle(float pwm_duty) override;
  float getPWMDutyCycle() override;
  int32_t getEncoderCnt() override;
  void resetEncoderCnt() override;
  float getWindingCurrent() override;

 private:
  hFramework::hMotor& motor_;
  const bool reverse_polarity_;
  float pwm_duty_ = 0.0F;
};

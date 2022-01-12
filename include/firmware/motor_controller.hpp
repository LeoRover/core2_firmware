#pragma once

#include <hMotor.h>

#include <stdint.h>

struct MotorConfiguration {
  hFramework::hMotor& motor;
  bool reverse_polarity;
};

class MotorController {
 public:
  MotorController(const MotorConfiguration& motor_conf)
      : motor_(motor_conf.motor),
        reverse_polarity_(motor_conf.reverse_polarity){};

  /**
   * Initialize the Motor Controller.
   */
  void init();

  /**
   * Set the PWM Duty Cycle to the motor driver
   * @param pwm_duty The PWM Duty Cycle in percents
   */
  void setPWMDutyCycle(float pwm_duty);

  /**
   * Get the current PWM Duty Cycle
   */
  float getPWMDutyCycle();

  /**
   * Get the number of encoder ticks.
   * @return encoder ticks
   */
  int32_t getEncoderCnt();

  /**
   * Set the number of encoder ticks to 0.
   */
  void resetEncoderCnt();

  /**
   * Get motor winding current in Amperes
   */
  float getWindingCurrent();

 private:
  hFramework::hMotor& motor_;
  const bool reverse_polarity_;
  float pwm_duty_ = 0.0F;
};

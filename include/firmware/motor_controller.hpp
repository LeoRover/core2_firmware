#pragma once

#include <hMotor.h>

#include <stdint.h>

struct MotorConfiguration {
  hFramework::hMotor& motor;
};

class MotorController {
 public:
  MotorController(const MotorConfiguration& motor_conf)
      : motor_(motor_conf.motor){};

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

  /**
   * Set motor polarity.
   * @param polarity motor polarity
   */
  void setMotorPolarity(Polarity polarity);

  /**
   * Set encoder polarity.
   * @param polarity encoder polarity
   */
  void setEncoderPolarity(Polarity polarity);

 private:
  hFramework::hMotor& motor_;
  float pwm_duty_ = 0.0F;
};
